import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import serial.tools.list_ports

# Configurações
MAX_POINTS = 100
BAUD_RATE = 115200

def encontrar_porta_esp32():
    for p in serial.tools.list_ports.comports():
        if "USB" in p.description or "CP210" in p.description:
            return p.device
    return None

# Inicialização
porta = encontrar_porta_esp32()
if not porta:
    print("Erro: ESP32 não encontrado.")
    exit()

ser = serial.Serial(porta, BAUD_RATE)

tensao_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
corrente_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# Armazena apenas o índice absoluto global do evento
# Deque descarta eventos antigos automaticamente
blackout_indices = deque() 
surtos_indices = deque()
global_counter = 0 # Contador para marcar o tempo exato do evento

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
line1, = ax1.plot([], [], 'b-', lw=2)
line2, = ax2.plot([], [], 'r-', lw=2)
scat_blackout = ax1.plot([], [], 'kx', markersize=10, zorder=3)[0]
scat_surtos = ax1.plot([], [], 'o', color='orange', markersize=8, zorder=3)[0]

def update(frame):
    global global_counter
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("Data_python,"):
                dados = line.split(',')
                if len(dados) == 5:
                    v, i, b, s = float(dados[1]), float(dados[2]), int(dados[3]), int(dados[4])
                    
                    tensao_data.append(v)
                    corrente_data.append(i)
                    global_counter += 1

                    # Registra o evento
                    if b: blackout_indices.append(global_counter)
                    if s: surtos_indices.append(global_counter)

                    # Remove eventos que já saíram da janela de visualização (MAX_POINTS atrás)
                    while blackout_indices and blackout_indices[0] <= global_counter - MAX_POINTS:
                        blackout_indices.popleft()
                    while surtos_indices and surtos_indices[0] <= global_counter - MAX_POINTS:
                        surtos_indices.popleft()

                    # Renderização
                    line1.set_data(range(MAX_POINTS), tensao_data)
                    line2.set_data(range(MAX_POINTS), corrente_data)

                    # Converte índice global para índice local (0 a 99)
                    # O evento local = índice_evento - (global_counter - MAX_POINTS)
                    offset = global_counter - MAX_POINTS
                    scat_blackout.set_data([idx - offset for idx in blackout_indices], 
                                         [tensao_data[idx - offset - 1] for idx in blackout_indices])
                    
                    scat_surtos.set_data([idx - offset for idx in surtos_indices], 
                                       [tensao_data[idx - offset - 1] for idx in surtos_indices])

                    ax1.set_ylim(0, max(max(tensao_data), 250) * 1.2)
                    ax2.set_ylim(0, max(max(corrente_data), 50) * 1.2)
        except: pass
    return line1, line2, scat_blackout, scat_surtos

ani = FuncAnimation(fig, update, interval=30, blit=True)
plt.show()
