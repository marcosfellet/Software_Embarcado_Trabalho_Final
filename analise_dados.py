import serial
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import serial.tools.list_ports
import time

# --- CONFIGURAÇÕES ---
SIMULACAO = True # Mude para False quando estiver com o ESP32 conectado
MAX_POINTS = 100
BAUD_RATE = 115200

def encontrar_porta_esp32():
    for p in serial.tools.list_ports.comports():
        if "USB" in p.description or "CP210" in p.description:
            return p.device
    return None

# --- INICIALIZAÇÃO DA PORTA SERIAL ---
ser = None
if not SIMULACAO:
    porta = encontrar_porta_esp32()
    if not porta:
        print("Erro: ESP32 não encontrado.")
        exit()
    ser = serial.Serial(porta, BAUD_RATE)

# --- CONTROLE DE TEMPO RELATIVO ---
tempo_inicio = time.time() # Guarda o segundo exato em que o script iniciou

# Inicializa o eixo X com valores negativos para a linha começar preenchida na tela
tempo_data = deque([-(MAX_POINTS - i) * 0.1 for i in range(MAX_POINTS)], maxlen=MAX_POINTS)
tensao_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
corrente_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# Armazenará tuplas: (segundos_do_evento, valor_da_tensao)
blackout_events = deque() 
surtos_events = deque()

# Configuração visual
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
line1, = ax1.plot([], [], 'b-', lw=1.5)
line2, = ax2.plot([], [], 'r-', lw=1.5)
scat_blackout = ax1.plot([], [], 'kx', markersize=10, zorder=3)[0]
scat_surtos = ax1.plot([], [], 'o', color='orange', markersize=8, zorder=3)[0]

ax1.set_title("Monitoramento da Rede (Tempo Relativo)")
ax1.set_ylabel("Tensão (V)")
ax2.set_ylabel("Corrente (A)")

# Nomeia o eixo X indicando que são segundos acumulados
ax2.set_xlabel("Tempo de Medição (segundos)")

ax1.set_ylim(0, 300)
ax2.set_ylim(0, 100)

def update(_):
    line = None
    
    if SIMULACAO:
        v = random.uniform(210, 230)
        i = random.uniform(10, 20)
        b = 1 if random.random() > 0.95 else 0
        s = 1 if random.random() > 0.95 else 0
        line = f"Data_python,{v:.2f},{i:.2f},{b},{s}"
    else:
        if ser and ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()

    if line and line.startswith("Data_python,"):
        dados = line.split(',')
        if len(dados) == 5:
            try:
                v, i, b, s = float(dados[1]), float(dados[2]), int(dados[3]), int(dados[4])
                
                # Calcula quantos segundos se passaram desde o início do script
                tempo_atual = time.time() - tempo_inicio 
                
                tempo_data.append(tempo_atual)
                tensao_data.append(v)
                corrente_data.append(i)

                if b: blackout_events.append((tempo_atual, v))
                if s: surtos_events.append((tempo_atual, v))
                
                # Limpa eventos antigos usando o tempo relativo do primeiro dado da janela
                tempo_mais_antigo = tempo_data[0]
                while blackout_events and blackout_events[0][0] < tempo_mais_antigo: blackout_events.popleft()
                while surtos_events and surtos_events[0][0] < tempo_mais_antigo: surtos_events.popleft()

                line1.set_data(tempo_data, tensao_data)
                line2.set_data(tempo_data, corrente_data)
                
                if blackout_events:
                    tempos, valores = zip(*blackout_events)
                    scat_blackout.set_data(tempos, valores)
                else:
                    scat_blackout.set_data([], [])
                    
                if surtos_events:
                    tempos, valores = zip(*surtos_events)
                    scat_surtos.set_data(tempos, valores)
                else:
                    scat_surtos.set_data([], [])

                # Ajusta os limites horizontais com base nos segundos calculados
                ax1.set_xlim(tempo_data[0], tempo_data[-1])
                ax2.set_xlim(tempo_data[0], tempo_data[-1])
                
            except ValueError: pass

    return line1, line2, scat_blackout, scat_surtos

ani = FuncAnimation(fig, update, interval=100, blit=False)
plt.tight_layout()
plt.show()
