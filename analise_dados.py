import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np
import math

# Valores iniciais de tensão e corrente para os eixos y do gráfico
TENSAO  = 220
CORRENTE = 63
ADICIONAL = 5


# Configurações da porta serial
SERIAL_PORT = 'COM3'  # Substitua pelo nome da porta serial correta
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Estrutura de dados
max_points = 100
max_falhas = 10
tensao_data = deque([0]*max_points, maxlen=max_points)
corrente_data = deque([0]*max_points, maxlen=max_points)


# Configurção dos Gráficos
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
line1, = ax1.plot([], [], 'b-', lw=2, zorder=1) 
scat_blackout = ax1.plot([], [], 'ko', markersize=8, zorder=2)[0]
line2, = ax2.plot([], [], 'r-', lw = 2) # Corrente

# Inicialize os marcadores de eventos 
scat_blackout = ax1.plot([], [], 'ko', markersize=8, label='Blackout')[0]
scat_surtos = ax1.plot([], [], 'o', color='orange', markersize=8, label='Surtos')[0]

# Armazena os blackouts e os surtos

blackout_events = [] # Lista de tuplas (x, y)
surtos_events = []   # Lista de tuplas (x, y)

# Configuração visual dos eixos

ax1.set_title("Monitoramento da Rede em Tempo Real")
ax1.set_ylabel("Tensão(V)")
ax1.set_ylim(0, TENSAO)
ax2.set_ylabel("Corrente (A)")
ax2.set_ylim(0, CORRENTE)


def update(frame):
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            dados = line.split(',')
            
            if len(dados) == 4:
                v = float(dados[0])
                i = float(dados[1])
                blackout = int(dados[2])
                surtos = int(dados[3])
                
                tensao_data.append(v)
                corrente_data.append(i)

                line1.set_data(range(max_points), tensao_data)
                line2.set_data(range(max_points), corrente_data)

                # 1. Se ocorreu evento, salva a coordenada atual

                if(blackout != 0):
                    blackout_events.append([max_points - 1, v])

                if(surtos != 0):
                    surtos_events.append([max_points - 1, v])

                # 2. Faz os pontos antigos "andarem" para a esquerda
                for pt in blackout_events: 
                    pt[0] -= 1

                for pt in surtos_events: 
                    pt[0] -= 1

                # 3. Remove pontos que saíram da tela (X < 0)
                blackout_events[:] = [pt for pt in blackout_events if pt[0] >= 0]

                surtos_events[:] = [pt for pt in surtos_events if pt[0] >= 0]

                # 4. Atualiza os marcadores (separando X e Y das listas de tuplas)
                if blackout_events:
                    scat_blackout.set_data([pt[0] for pt in blackout_events], [pt[1] for pt in blackout_events])
                else:
                    scat_blackout.set_data([], [])
                    
                if surtos_events:
                    scat_surtos.set_data([pt[0] for pt in surtos_events], [pt[1] for pt in surtos_events])
                else:
                    scat_surtos.set_data([], [])

                
                # --- Auto-Scaling Dinâmico ---
                
                # Encontra o maior valor no buffer atual e adiciona 20% de margem
                max_v = max(tensao_data) if max(tensao_data) > 0 else 250
                max_i = max(corrente_data) if max(corrente_data) > 0 else 50
                
                # Define os novos limites
                ax1.set_ylim(0, max_v * 1.2)
                ax2.set_ylim(0, max_i * 1.2)
                
                # ----------------------------------------
        except (ValueError, IndexError):
            pass

    return line1, line2, scat_blackout, scat_surtos
    
ani = FuncAnimation(fig, update, interval = 20, blit = True)
plt.tight_layout()
plt.show


