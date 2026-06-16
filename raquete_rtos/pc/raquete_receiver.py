"""
Raquete Instrumentada — Receptor Python
Recebe frames da BlackPill via USB-Serial (ou HC-12) e exibe:
  - Gráfico em tempo real dos 6 piezos e IMU
  - Heatmap acumulado de regiões de impacto

Dependências:
    pip install pyserial matplotlib numpy

Uso:
    python raquete_receiver.py              # detecta porta automaticamente
    python raquete_receiver.py COM3         # Windows
    python raquete_receiver.py /dev/ttyUSB0 # Linux
"""

import sys
import struct
import threading
import queue
import time
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import warnings
warnings.filterwarnings("ignore")

# ---------------------------------------------------------------------------
# Protocolo
# ---------------------------------------------------------------------------
FRAME_START  = 0xAA
FRAME_END    = 0x55
FRAME_SIZE   = 32
BAUD_RATE    = 115200     # mesmo baud do uart_tx.c (USART1)
N_PIEZOS     = 6
N_REGIOES    = 9

# struct.unpack format — little-endian:
# B=uint8 I=uint32 f=float H=uint16
# [ len(B) | ts(I) | region(B) | accel(f) | ang_vel(f) | angle(f) | adc x6(H) ]
FRAME_FORMAT = '<BIBfff6H'
FRAME_PAYLOAD = struct.calcsize(FRAME_FORMAT)  # 30 bytes

# ---------------------------------------------------------------------------
# Nomes das regiões para o heatmap
# ---------------------------------------------------------------------------
REGION_NAMES = [
    "Centro",
    "Borda\nSuperior",
    "Borda\nDireita",
    "Borda\nInferior",
    "Borda\nEsquerda",
    "Canto\nSup-Dir",
    "Canto\nSup-Esq",
    "Canto\nInf-Dir",
    "Canto\nInf-Esq",
]

# posição de cada região no grid 3x3 do heatmap (linha, coluna)
REGION_GRID = {
    0: (1, 1),  # centro
    1: (0, 1),  # borda superior
    2: (1, 2),  # borda direita
    3: (2, 1),  # borda inferior
    4: (1, 0),  # borda esquerda
    5: (0, 2),  # canto sup-dir
    6: (0, 0),  # canto sup-esq
    7: (2, 2),  # canto inf-dir
    8: (2, 0),  # canto inf-esq
}

# ---------------------------------------------------------------------------
# Detecção automática de porta serial
# ---------------------------------------------------------------------------
def detectar_porta():
    portas = serial.tools.list_ports.comports()
    candidatos = [p for p in portas if any(k in p.description.upper()
                  for k in ["CH340", "CP210", "USB", "UART", "SERIAL"])]
    if candidatos:
        return candidatos[0].device
    if portas:
        return portas[0].device
    return None

# ---------------------------------------------------------------------------
# Thread de leitura serial — roda em background
# ---------------------------------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, porta, data_queue):
        super().__init__(daemon=True)
        self.porta      = porta
        self.data_queue = data_queue
        self.running    = True
        self.frames_ok  = 0
        self.frames_err = 0

    def run(self):
        try:
            ser = serial.Serial(self.porta, BAUD_RATE, timeout=1.0)
            print(f"[Serial] Conectado em {self.porta} @ {BAUD_RATE} baud")
        except serial.SerialException as e:
            print(f"[ERRO] Não foi possível abrir {self.porta}: {e}")
            return

        buf = bytearray()

        while self.running:
            try:
                chunk = ser.read(64)
                if not chunk:
                    continue
                buf.extend(chunk)

                # procura frames válidos no buffer
                while len(buf) >= FRAME_SIZE:
                    # localiza byte de início
                    idx = buf.find(FRAME_START)
                    if idx == -1:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]  # descarta lixo antes do frame

                    if len(buf) < FRAME_SIZE:
                        break

                    # verifica byte de fim na posição esperada
                    if buf[FRAME_SIZE - 1] != FRAME_END:
                        del buf[0]     # byte de início falso — avança 1
                        self.frames_err += 1
                        continue

                    # extrai payload (entre start e end)
                    payload = bytes(buf[1:FRAME_SIZE - 1])
                    del buf[:FRAME_SIZE]

                    try:
                        parsed = struct.unpack(FRAME_FORMAT, payload[:FRAME_PAYLOAD])
                        frame = {
                            'length'      : parsed[0],
                            'timestamp_ms': parsed[1],
                            'region_id'   : parsed[2],
                            'peak_accel_g': parsed[3],
                            'ang_vel_dps' : parsed[4],
                            'angle_deg'   : parsed[5],
                            'adc'         : list(parsed[6:12]),
                        }
                        self.data_queue.put(frame)
                        self.frames_ok += 1
                    except struct.error as e:
                        self.frames_err += 1

            except serial.SerialException:
                print("[Serial] Conexão perdida. Aguardando...")
                time.sleep(1)

        ser.close()

# ---------------------------------------------------------------------------
# Dashboard de visualização
# ---------------------------------------------------------------------------
class Dashboard:
    HISTORY = 50   # janela de amostras no gráfico de séries temporais

    def __init__(self, reader):
        self.reader     = reader
        self.data_queue = reader.data_queue

        # histórico para gráficos de série temporal
        self.ts_history      = []
        self.accel_history   = []
        self.angvel_history  = []
        self.angle_history   = []
        self.adc_history     = [[] for _ in range(N_PIEZOS)]

        # contadores do heatmap
        self.hit_count = np.zeros(N_REGIOES, dtype=int)

        self._setup_figure()

    def _setup_figure(self):
        self.fig = plt.figure(figsize=(14, 8), facecolor='#1a1a2e')
        self.fig.canvas.manager.set_window_title("Raquete Instrumentada — Monitor")
        gs = gridspec.GridSpec(3, 2, figure=self.fig,
                               left=0.07, right=0.97,
                               top=0.93, bottom=0.08,
                               hspace=0.45, wspace=0.35)

        style = dict(facecolor='#16213e')

        # [0,0] — aceleração de pico
        self.ax_accel = self.fig.add_subplot(gs[0, 0], **style)
        self.ax_accel.set_title("Aceleração de pico (g)", color='white', fontsize=9)
        self.ax_accel.tick_params(colors='gray', labelsize=7)
        self.line_accel, = self.ax_accel.plot([], [], color='#e94560', lw=1.5)
        self.ax_accel.set_ylim(0, 20)
        self.ax_accel.set_facecolor('#0d1b2a')

        # [1,0] — velocidade angular
        self.ax_angvel = self.fig.add_subplot(gs[1, 0], **style)
        self.ax_angvel.set_title("Velocidade angular (°/s)", color='white', fontsize=9)
        self.ax_angvel.tick_params(colors='gray', labelsize=7)
        self.line_angvel, = self.ax_angvel.plot([], [], color='#0f3460', lw=1.5)
        self.ax_angvel.set_facecolor('#0d1b2a')

        # [2,0] — ângulo
        self.ax_angle = self.fig.add_subplot(gs[2, 0], **style)
        self.ax_angle.set_title("Ângulo de incidência (°)", color='white', fontsize=9)
        self.ax_angle.tick_params(colors='gray', labelsize=7)
        self.line_angle, = self.ax_angle.plot([], [], color='#53d8fb', lw=1.5)
        self.ax_angle.set_ylim(-180, 180)
        self.ax_angle.set_facecolor('#0d1b2a')

        # [0:2, 1] — piezos (6 canais)
        self.ax_piezos = self.fig.add_subplot(gs[0:2, 1], **style)
        self.ax_piezos.set_title("Piezos ADC — 6 canais", color='white', fontsize=9)
        self.ax_piezos.tick_params(colors='gray', labelsize=7)
        colors_piezo = ['#e94560','#f5a623','#7ed321','#53d8fb','#bd10e0','#ffffff']
        self.lines_piezo = []
        for i in range(N_PIEZOS):
            line, = self.ax_piezos.plot([], [], color=colors_piezo[i],
                                        lw=1.2, label=f'P{i}')
            self.lines_piezo.append(line)
        self.ax_piezos.set_ylim(0, 4096)
        self.ax_piezos.legend(loc='upper right', fontsize=7,
                              facecolor='#1a1a2e', labelcolor='white')
        self.ax_piezos.set_facecolor('#0d1b2a')

        # [2, 1] — heatmap de regiões
        self.ax_heat = self.fig.add_subplot(gs[2, 1], **style)
        self.ax_heat.set_title("Heatmap de impactos", color='white', fontsize=9)
        self.ax_heat.set_facecolor('#0d1b2a')
        self.ax_heat.set_xlim(-0.5, 2.5)
        self.ax_heat.set_ylim(-0.5, 2.5)
        self.ax_heat.set_aspect('equal')
        self.ax_heat.axis('off')

        self.heat_patches = {}
        self.heat_texts   = {}
        for r, (row, col) in REGION_GRID.items():
            rect = plt.Rectangle((col - 0.45, row - 0.45), 0.9, 0.9,
                                  color='#0d1b2a', ec='#0f3460', lw=1)
            self.ax_heat.add_patch(rect)
            txt = self.ax_heat.text(col, row, f"{REGION_NAMES[r]}\n0",
                                    ha='center', va='center',
                                    color='white', fontsize=6)
            self.heat_patches[r] = rect
            self.heat_texts[r]   = txt

        self.norm = Normalize(vmin=0, vmax=1)
        self.cmap = plt.colormaps['YlOrRd']

        # título geral
        self.title = self.fig.suptitle("Aguardando dados...",
                                        color='white', fontsize=11,
                                        fontweight='bold')

    def _update_heatmap(self):
        max_hits = max(self.hit_count.max(), 1)
        for r, patch in self.heat_patches.items():
            intensity = self.hit_count[r] / max_hits
            patch.set_color(self.cmap(self.norm(intensity)))
            self.heat_texts[r].set_text(
                f"{REGION_NAMES[r]}\n{self.hit_count[r]}"
            )
            # contraste do texto
            self.heat_texts[r].set_color('black' if intensity > 0.5 else 'white')

    def update(self, frame_num):
        # drena todos os frames disponíveis
        novos = 0
        while not self.data_queue.empty():
            try:
                f = self.data_queue.get_nowait()
            except queue.Empty:
                break

            # acumula histórico
            self.ts_history.append(f['timestamp_ms'] / 1000.0)
            self.accel_history.append(f['peak_accel_g'])
            self.angvel_history.append(f['ang_vel_dps'])
            self.angle_history.append(f['angle_deg'])
            for i in range(N_PIEZOS):
                self.adc_history[i].append(f['adc'][i])

            # heatmap
            rid = f['region_id']
            if 0 <= rid < N_REGIOES:
                self.hit_count[rid] += 1

            novos += 1

        if novos == 0:
            return

        # mantém só os últimos HISTORY pontos
        if len(self.ts_history) > self.HISTORY:
            self.ts_history    = self.ts_history[-self.HISTORY:]
            self.accel_history = self.accel_history[-self.HISTORY:]
            self.angvel_history= self.angvel_history[-self.HISTORY:]
            self.angle_history = self.angle_history[-self.HISTORY:]
            for i in range(N_PIEZOS):
                self.adc_history[i] = self.adc_history[i][-self.HISTORY:]

        ts = self.ts_history

        # atualiza linhas
        self.line_accel.set_data(ts, self.accel_history)
        self.ax_accel.set_xlim(ts[0], ts[-1] + 0.1)
        self.ax_accel.set_ylim(0, max(max(self.accel_history) * 1.2, 5))

        self.line_angvel.set_data(ts, self.angvel_history)
        self.ax_angvel.set_xlim(ts[0], ts[-1] + 0.1)
        self.ax_angvel.set_ylim(0, max(max(self.angvel_history) * 1.2, 100))

        self.line_angle.set_data(ts, self.angle_history)
        self.ax_angle.set_xlim(ts[0], ts[-1] + 0.1)

        for i in range(N_PIEZOS):
            self.lines_piezo[i].set_data(ts, self.adc_history[i])
        self.ax_piezos.set_xlim(ts[0], ts[-1] + 0.1)

        self._update_heatmap()

        # título com estatísticas
        total = self.hit_count.sum()
        fps   = self.reader.frames_ok
        self.title.set_text(
            f"Raquete Instrumentada  |  Impactos: {total}  |  "
            f"Frames OK: {fps}  |  Erros: {self.reader.frames_err}"
        )

        self.fig.canvas.draw_idle()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    # porta serial
    if len(sys.argv) > 1:
        porta = sys.argv[1]
    else:
        porta = detectar_porta()
        if porta is None:
            print("[ERRO] Nenhuma porta serial encontrada.")
            print("       Uso: python raquete_receiver.py <PORTA>")
            sys.exit(1)
        print(f"[Auto] Porta detectada: {porta}")

    # fila de comunicação entre thread serial e dashboard
    data_queue = queue.Queue(maxsize=200)

    # inicia thread de leitura serial
    reader = SerialReader(porta, data_queue)
    reader.start()

    # dashboard matplotlib
    dash = Dashboard(reader)

    print("[Info] Dashboard aberto. Feche a janela para encerrar.")
    print(f"[Info] Protocolo: {FRAME_SIZE} bytes/frame @ {BAUD_RATE} baud")
    print("[Info] Aguardando frames da raquete...\n")

    # animação — atualiza a cada 100 ms
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(dash.fig, dash.update, interval=100, cache_frame_data=False)

    plt.show()

    reader.running = False
    print(f"\n[Info] Encerrado. Frames recebidos: {reader.frames_ok} | Erros: {reader.frames_err}")

if __name__ == "__main__":
    main()
