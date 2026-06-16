# Raquete Instrumentada — Funcionamento do Sistema

Firmware FreeRTOS para STM32F411 (BlackPill) que instrumenta uma raquete com
6 piezos + IMU BNO08x, processa os impactos, mostra um heatmap no display
ST7789 e transmite cada impacto ao vivo para o PC via UART. No PC, um programa
Python recebe os frames e exibe um dashboard em tempo real.

Este documento descreve **como as tarefas se organizam e funcionam** e **o
software Python que apresenta as informações no PC**. Toda afirmação referencia
o arquivo e a linha onde está implementada.

---

## 1. Visão geral

O firmware roda 5 tarefas FreeRTOS criadas em `src/main.c:118-122`, com o
scheduler iniciado em `src/main.c:124`. As tarefas se comunicam **somente** por
filas e por 3 snapshots globais — nenhuma compartilha estado por outro meio.

```
          ADC1+DMA (background)              BNO08x (I2C1)
              │                                  │
              ▼                                  ▼
        ┌───────────┐   snapshots globais  ┌───────────┐
        │ T1 Piezo  │ ◄─── angle/vel/accel ─┤ T2 IMU    │
        │ prio 4    │                       │ prio 4    │
        │ 5 ms      │                       │ 5 ms      │
        └─────┬─────┘                       └───────────┘
              │ ImpactRaw_t
              ▼  xQueueImpacto (8)
        ┌───────────┐
        │ T3 Proc   │  estima região pelo piezo de maior amplitude
        │ prio 3    │
        └──┬─────┬──┘
   ImpactPacket │ RegionHit_t
   xQueueTX(4)  │ xQueueDisplay(1, overwrite)
        ▼       ▼
  ┌─────────┐ ┌──────────┐
  │ T4 TX   │ │ T5 Disp  │  heatmap 3x3 no ST7789
  │ prio 2  │ │ prio 1   │
  └────┬────┘ │ 50 ms    │
       │      └──────────┘
       ▼ frame 32 B @115200
   USART1 (PA9) ──► PC ──► raquete_receiver.py (dashboard matplotlib)
```

Periféricos inicializados antes do scheduler em `src/main.c:107-109`
(`uart1_init`, `adc_dma_init`, `st7789_init`); a IMU é inicializada **dentro**
da T2 (`src/task_imu.c:23`) porque `bno08x_init()` usa `delay_ms`, que cede a
CPU ao scheduler.

Clock: HSI → PLL → 96 MHz (`src/main.c:54-75`). O SysTick pertence ao
FreeRTOS; o delay de microssegundos usa o cycle counter DWT
(`src/main.c:39-43`), e `delay_ms` cede a CPU via `vTaskDelay` quando o
scheduler está rodando (`src/main.c:45-51`).

---

## 2. As 5 tarefas

| # | Tarefa | Função | Prioridade | Stack | Cadência | Fonte |
|---|--------|--------|:----------:|:-----:|----------|-------|
| T1 | `vTaskAquisicaoPiezos` | Lê os 6 piezos | 4 | 256 | Periódica 5 ms | `src/task_piezo.c` |
| T2 | `vTaskFusaoIMU` | Lê ângulo/giro/aceleração da IMU | 4 | 512 | Periódica 5 ms | `src/task_imu.c` |
| T3 | `vTaskProcessamento` | Estima região e distribui | 3 | 512 | Bloqueada na fila | `src/task_proc.c` |
| T4 | `vTaskTransmissao` | Serializa e envia por UART | 2 | 512 | Bloqueada na fila | `src/task_tx.c` |
| T5 | `vTaskDisplay` | Desenha o heatmap | 1 | 512 | Periódica 50 ms | `src/task_display.c` |

Prioridades e tamanhos de stack definidos em `src/main.c:118-122`. Quanto maior
o número, maior a prioridade (FreeRTOS).

### T1 — Aquisição de piezos (prio 4, 5 ms)

O DMA mantém o buffer do ADC atualizado em background; a tarefa só tira
snapshots. Inicia o scan livre em `src/task_piezo.c:22` (`adc_dma_start`) e a
cada 5 ms (`src/task_piezo.c:37`, `vTaskDelayUntil`):

1. Marca timestamp e `source = 0` (`src/task_piezo.c:26-27`).
2. Copia os 6 canais do ADC (`adc_snapshot`, `src/task_piezo.c:29`).
3. **Anexa** os snapshots da IMU escritos por T2 — ângulo, velocidade angular
   e aceleração (`src/task_piezo.c:30-32`).
4. Envia o `ImpactRaw_t` para a fila `xQueueImpacto` com timeout 0 — se a fila
   estiver cheia, **descarta** a leitura (`src/task_piezo.c:35`).

O período fixo (`vTaskDelayUntil`) existe para ceder CPU; sem ele a T5 (display)
ficaria faminta (`src/task_piezo.c:4-7`).

### T2 — Fusão IMU / BNO08x (prio 4, 5 ms)

O BNO085 (`0x4A`, protocolo SHTP) faz a fusão **internamente** e entrega
ângulo (rotation vector → euler) e velocidade angular prontos. A T2 **não envia
frames próprios**: a cada ciclo drena até 8 pacotes pendentes, guardando o mais
recente (`src/task_imu.c:34-35`), e atualiza os 3 snapshots globais
(`src/task_imu.c:37-41`):

- `imu_angle_snapshot` ← `data.pitch` (eixo da raquete)
- `imu_angvel_snapshot` ← `data.speed` (°/s)
- `imu_accel_snapshot` ← `data.accel_g` (g)

Esses snapshots são `volatile float` em `src/freertos_objects.c:7-9`, escritos
por T2 e lidos por T1. Esse desenho evita que a IMU dispute a `xQueueImpacto`
com o piezo — em ~200 frames/s o piezo enchia a fila e descartava os frames da
IMU (`src/task_imu.c:5-9`).

### T3 — Processamento (prio 3, dirigida por evento)

Bloqueia em `xQueueReceive(..., portMAX_DELAY)` esperando um `ImpactRaw_t`
(`src/task_proc.c:39`). Para cada item:

1. **Estima a região** pelo canal ADC de maior amplitude
   (`estimar_regiao`, `src/task_proc.c:17-30`). O mapa canal→região é
   `{0,1,2,3,4,5}` e deve ser calibrado experimentalmente
   (`src/task_proc.c:19`).
2. Monta o `ImpactPacket_t` (timestamp, região, g de pico, velocidade angular,
   ângulo e os 6 picos de ADC) — `src/task_proc.c:41-47`.
3. Envia o pacote para **T4** via `xQueueTX` (`src/task_proc.c:49`).
4. Monta um `RegionHit_t` com intensidade normalizada
   (`peak_accel_g / 16.0`, saturada em 1.0; `src/task_proc.c:51-54`) e o envia
   para **T5** via `xQueueOverwrite` — que nunca bloqueia e mantém sempre o
   item mais recente (`src/task_proc.c:55`).

### T4 — Transmissão UART (prio 2, dirigida por evento)

Bloqueia em `xQueueTX` (`src/task_tx.c:25`) e serializa cada `ImpactPacket_t`
num frame binário de 32 bytes (`src/task_tx.c:27-37`), enviado por USART1
(PA9, 115200 8N1) com `uart1_send` (`src/task_tx.c:39`). Layout do frame na
seção 5.

### T5 — Display heatmap (prio 1, 50 ms)

Inicializa o heatmap (`heat_init`, `src/task_display.c:21`) e a cada 50 ms
(`src/task_display.c:38`):

1. **Drena** todos os `RegionHit_t` pendentes sem bloquear
   (`src/task_display.c:25`), incrementando o contador por região e atualizando
   o máximo (`src/task_display.c:26-30`).
2. Para cada uma das 9 regiões, calcula a intensidade relativa
   (`hit_count[r] / max_hits`) e redesenha a célula
   (`heat_draw_region`, `src/task_display.c:33-36`).

O heatmap é uma grade 3x3 no ST7789 240×240, célula de 76 px + borda
(`src/heatmap.c:10`). A cor vai de preto/azul (frio) a vermelho (quente) em 6
faixas (`heat_color`, `src/heatmap.c:26-33`). O mapa região→(linha,coluna) é o
**mesmo** do receiver Python (`src/heatmap.c:14-24`).

---

## 3. Como as tarefas se comunicam

### Filas (criadas em `src/freertos_objects.c:11-17`)

| Fila | Produtor → Consumidor | Item | Tamanho | Política |
|------|-----------------------|------|:-------:|----------|
| `xQueueImpacto` | T1 → T3 | `ImpactRaw_t` | 8 | Descarta se cheia (timeout 0) |
| `xQueueTX` | T3 → T4 | `ImpactPacket_t` | 4 | Descarta se cheia (timeout 0) |
| `xQueueDisplay` | T3 → T5 | `RegionHit_t` | 1 | `xQueueOverwrite` (sempre o último) |

### Snapshots globais (T2 → T1)

`imu_angle_snapshot`, `imu_angvel_snapshot`, `imu_accel_snapshot` —
`volatile float` declarados em `src/freertos_objects.c:7-9` e externados em
`src/freertos_objects.h:36-38`. Caminho deliberadamente fora das filas para a
IMU não competir com o piezo.

### Estruturas de dados (`src/freertos_objects.h`)

- **`ImpactRaw_t`** (`:8-15`) — leituras brutas + snapshots da IMU. T1 → T3.
- **`ImpactPacket_t`** (`:18-25`) — impacto já com região estimada. T3 → T4.
- **`RegionHit_t`** (`:28-32`) — região + intensidade normalizada (0–1). T3 → T5.

### Regiões (`N_REGIOES = 9`, `src/board.h:35`)

`0` = centro, `1-4` = bordas, `5-8` = cantos (`src/board.h:32-34`). Há 6 piezos
(`N_PIEZOS = 6`, `src/board.h:26`), então hoje apenas 6 das 9 regiões são
diretamente endereçadas pelo mapa de `src/task_proc.c:19`.

---

## 4. Diagnóstico visual (cores de falha)

Quando algo falha, a tela inteira é pintada com uma cor e o sistema trava — útil
para diagnóstico sem debugger:

| Cor | Significado | Fonte |
|-----|-------------|-------|
| Magenta | Heap esgotada (`mallocFailed`) | `src/main.c:83-86` |
| Laranja | Stack overflow | `src/main.c:87-91` |
| Branco | HardFault | `src/main.c:94` |
| Amarelo | BusFault | `src/main.c:95` |
| Ciano | UsageFault | `src/main.c:96` |
| Cinza | MemManage | `src/main.c:97` |
| Retângulo vermelho (canto) | Scheduler retornou | `src/main.c:127` |

O idle hook entra em `__WFI()` para baixo consumo entre eventos
(`src/main.c:78-80`).

---

## 5. Protocolo UART (frame de 32 bytes)

Montado em `src/task_tx.c:27-37`, todos os campos em **little-endian**:

```
offset  tam  campo
  0       1   FRAME_START   = 0xAA
  1       1   length        = 30 (payload sem start/end)
  2       4   timestamp_ms  (uint32)
  6       1   region_id     (uint8)
  7       4   peak_accel_g  (float)
 11       4   angular_vel_dps (float)
 15       4   angle_deg     (float)
 19      12   adc[6]        (6 × uint16)
 31       1   FRAME_END     = 0x55
            ───
            32 bytes total
```

Constantes: `FRAME_START 0xAA`, `FRAME_END 0x55`, `FRAME_SIZE 32`
(`src/task_tx.c:15-17`). Baud 115200 (`uart_tx.c` / USART1).

---

## 6. Software Python no PC — `pc/raquete_receiver.py`

Recebe os frames pela serial e mostra um **dashboard matplotlib em tempo real**.

> Há duas cópias do script: `pc/raquete_receiver.py` é a **atual** (115200 baud,
> API matplotlib nova). A da raiz `raquete_receiver.py` está **defasada**
> (9600 baud, `plt.cm.get_cmap` antigo) — use a de `pc/`.

### Dependências e uso (`pc/raquete_receiver.py:7-14`)

```bash
pip install pyserial matplotlib numpy

python raquete_receiver.py              # detecta a porta automaticamente
python raquete_receiver.py COM3         # Windows
python raquete_receiver.py /dev/ttyUSB0 # Linux
```

A detecção automática procura portas cuja descrição contenha CH340/CP210/USB/
UART/SERIAL e cai na primeira disponível (`detectar_porta`,
`pc/raquete_receiver.py:78-86`).

### Arquitetura: 2 threads + fila

Espelha o desenho do firmware (produtor/consumidor desacoplados por fila):

1. **`SerialReader`** (`pc/raquete_receiver.py:91-160`) — thread em background
   (`daemon`). Lê 64 bytes por vez (`:112`), acumula num buffer e procura
   frames válidos: localiza o `0xAA` (`:120`), descarta lixo anterior (`:125`),
   confere o `0x55` na posição final (`:131`) e, se válido, desempacota o
   payload com `struct.unpack` no formato `'<BIBfff6H'` (`:44`, `:141`). Cada
   frame vira um dict e entra em `data_queue` (`:142-151`). Mantém contadores
   `frames_ok` / `frames_err` (`:152-154`).
2. **`Dashboard`** (`pc/raquete_receiver.py:165-336`) — drena a fila e atualiza
   os gráficos a cada 100 ms via `FuncAnimation` (`:369`).

A fila entre as duas é `queue.Queue(maxsize=200)` (`pc/raquete_receiver.py:354`).

### O que o dashboard mostra (`_setup_figure`, `:184-259`)

- **Aceleração de pico (g)** — série temporal (`:195-200`).
- **Velocidade angular (°/s)** — série temporal (`:203-207`).
- **Ângulo de incidência (°)** — série temporal, eixo −180…180 (`:210-215`).
- **Piezos ADC — 6 canais** — uma linha por piezo, 0…4096 (`:217-230`).
- **Heatmap de impactos** — grade 3x3 colorida por contagem acumulada por
   região, com nome e total em cada célula (`:232-251`).

A janela de série temporal guarda os últimos 50 pontos (`HISTORY = 50`,
`:166`, recortados em `:300-306`). O heatmap normaliza pela região mais atingida
(`_update_heatmap`, `:261-270`) usando o colormap `YlOrRd` (`:254`). O título
da figura mostra total de impactos, frames OK e erros
(`pc/raquete_receiver.py:328-334`).

### Consistência firmware ↔ PC

Estes valores **precisam** bater entre os dois lados, senão o decode quebra:

| Constante | Firmware | Python |
|-----------|----------|--------|
| Start/End | `0xAA` / `0x55` (`src/task_tx.c:15-16`) | `0xAA` / `0x55` (`pc/raquete_receiver.py:34-35`) |
| Tamanho do frame | 32 (`src/task_tx.c:17`) | 32 (`pc/raquete_receiver.py:36`) |
| Baud | 115200 (USART1) | 115200 (`pc/raquete_receiver.py:37`) |
| Layout do payload | `src/task_tx.c:31-36` | `'<BIBfff6H'` (`pc/raquete_receiver.py:44`) |
| N_PIEZOS / N_REGIOES | 6 / 9 (`src/board.h:26,35`) | 6 / 9 (`pc/raquete_receiver.py:38-39`) |
| Mapa região→grid 3x3 | `src/heatmap.c:14-24` | `REGION_GRID` (`pc/raquete_receiver.py:63-73`) |
