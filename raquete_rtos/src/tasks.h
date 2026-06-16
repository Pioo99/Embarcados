#pragma once

/* Tarefas FreeRTOS — Raquete Instrumentada */
void vTaskAquisicaoPiezos(void *pv);  /* T1 — prio 4, periódica 5 ms    */
void vTaskFusaoIMU(void *pv);         /* T2 — prio 4, periódica 5 ms    */
void vTaskProcessamento(void *pv);    /* T3 — prio 3, xQueueImpacto     */
void vTaskTransmissao(void *pv);      /* T4 — prio 2, xQueueTX          */
void vTaskDisplay(void *pv);          /* T5 — prio 1, periódica 50 ms   */
