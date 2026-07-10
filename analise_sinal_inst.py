import serial
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import serial.tools.list_ports
import struct


SIMULACAO = False       
BAUD_RATE = 921600              
JANELA_SEGUNDOS = 5.0            
TAXA_ENVIO = 5000.0 # taxa de envio dos pacotes

############################### 
##### MAPEAMENTO DO PROTOCOLO #
############################### 

TAG = b'SG'
PACOTE = "<fbb"          # float + 2 bytes (blackout, surto)
TAMANHO_PACOTE = struct.calcsize(PACOTE)   # 6
TAMANHO_PACOTE_TOTAL = len(TAG) + TAMANHO_PACOTE  # 8

def encontrar_porta_esp32():
    """Retorna o nome da porta onde o ESP32 está conectado."""
    for p in serial.tools.list_ports.comports():
        if "USB" in p.description or "CP210" in p.description:
            return p.device
    return None

ser = None
if not SIMULACAO:
    porta = encontrar_porta_esp32()
    if not porta:
        print("Erro: ESP32 não encontrado.")
        exit()
    ser = serial.Serial(porta, BAUD_RATE, timeout=0.1)

###############################
###### ESTRUTURAS DE DADOS ####
###############################

serial_buffer = bytearray() # Acumula bytes da UART
contador_de_eventos = 0 # Número de pacotes recebidos (usado para tempo)

tempo_data = deque()   # Tempo real (em segundos) de cada ponto
tensao_data = deque()  # Valores de tensão
blackout_events = deque() # Eventos de blackout (tempo, tensão)

###############################
#### CONFIGURAÇÃO DO GRÁFICO ##
###############################

fig, ax1 = plt.subplots(figsize=(10, 6))

line1, = ax1.plot([], [], color='turquoise', lw=1.5, label='Tensão Nominal')
scat_blackout = ax1.plot([], [], marker='x', color='red', linestyle='None',
                         markersize=10, zorder=3, label='Evento: Blackout')[0]

ax1.set_title("Monitoramento da Rede")
ax1.set_ylabel("Tensão (V)")
ax1.set_xlabel("Tempo (s)")
ax1.set_ylim(-350, 350)
ax1.legend(loc='upper left', shadow=False, frameon=True)

def update(_):
    """
    Atualiza o gráfico a cada frame
    """
    global contador_de_eventos, serial_buffer
    dados_novos = False

    if SIMULACAO:
        v = random.uniform(210, 230)
        b = 1 if random.random() > 0.95 else 0

        contador_de_eventos += 1
        # Calcula o tempo real a partir da contagem de pacotes
        tempo_real = contador_de_eventos / TAXA_ENVIO

        tempo_data.append(tempo_real)
        tensao_data.append(v)
        if b:
            blackout_events.append((tempo_real, v))
        dados_novos = True

    ###############################################
    ############ LEITURA REAL DA UART #############
    ###############################################

    else:
        if ser and ser.in_waiting > 0:
            serial_buffer.extend(ser.read(ser.in_waiting))

            # Processa todos os pacotes completos no buffer
            while len(serial_buffer) >= TAMANHO_PACOTE_TOTAL:
                idx = serial_buffer.find(TAG)
                if idx == -1:
                    # Não encontrou cabeçalho: limpa, preservando possível 'S' no fim
                    if serial_buffer[-1] == ord('S'):
                        serial_buffer = bytearray([ord('S')])
                    else:
                        serial_buffer.clear()
                    break

                if idx + TAMANHO_PACOTE_TOTAL <= len(serial_buffer):
                    pacote_dados = serial_buffer[idx+2 : idx+8]
                    try:
                        v, b, s = struct.unpack(PACOTE, pacote_dados)
                        contador_de_eventos += 1
                        # Tempo real = contador / taxa de envio
                        tempo_real = contador_de_eventos / TAXA_ENVIO

                        tempo_data.append(tempo_real)
                        tensao_data.append(v)
                        if b:
                            blackout_events.append((tempo_real, v))
                        dados_novos = True
                    except struct.error:
                        pass
                    # Remove o pacote processado do buffer
                    serial_buffer = serial_buffer[idx + TAMANHO_PACOTE_TOTAL:]
                else:
                    # Se pacote incompleto, aguarda mais dados
                    serial_buffer = serial_buffer[idx:]
                    break

    ###############################                
    ##### ATUALIZAÇÃO DO GRÁFICO ##
    ############################### 

    if dados_novos and tempo_data:
        # Define a janela deslizante baseada no último tempo
        limite_esquerdo = tempo_data[-1] - JANELA_SEGUNDOS

        # Remove dados fora da janela
        while tempo_data and tempo_data[0] < limite_esquerdo:
            tempo_data.popleft()
            tensao_data.popleft()
        while blackout_events and blackout_events[0][0] < limite_esquerdo:
            blackout_events.popleft()

        # Atualiza a linha da tensão
        line1.set_data(list(tempo_data), list(tensao_data))

        # Atualiza os pontos de blackout
        if blackout_events:
            tempos, valores = zip(*blackout_events)
            scat_blackout.set_data(tempos, valores)
        else:
            scat_blackout.set_data([], [])

        # Ajusta os limites do eixo X
        ax1.set_xlim(limite_esquerdo, limite_esquerdo + JANELA_SEGUNDOS)

    return line1, scat_blackout

# Cria a animação (atualização a cada 30 ms)
ani = FuncAnimation(fig, update, interval=30, blit=False)
plt.tight_layout()
plt.show()
