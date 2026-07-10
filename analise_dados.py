import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import serial.tools.list_ports
import struct

###############################
####### CONFIGURAÇÕES #########
###############################
SIMULACAO = False       
BAUD_RATE = 921600              # Sincronizado com o ESP32
JANELA_SEGUNDOS = 0.1           # 100ms mostra exatamente ~6 ciclos de 60Hz
TAXA_ENVIO = 5000.0             # Taxa de envio das amostras do ESP32

# Total de pontos que cabem na tela de 100ms (0.1 * 5000 = 500 pontos)
PONTOS_TELA = int(JANELA_SEGUNDOS * TAXA_ENVIO) 

TAG = b'SG'
PACOTE = "<fbb"          
TAMANHO_PACOTE = struct.calcsize(PACOTE)   
TAMANHO_PACOTE_TOTAL = len(TAG) + TAMANHO_PACOTE  

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
serial_buffer = bytearray() 

# Agora o histórico armazena tuplas contendo (tensao, blackout, surto)
historico_dados = deque(maxlen=PONTOS_TELA + 200) 

###############################
#### CONFIGURAÇÃO DO GRÁFICO ##
###############################
fig, ax1 = plt.subplots(figsize=(10, 6))

# Linha turquesa da senoide
line1, = ax1.plot([], [], color='turquoise', lw=2, label='Sinal de Tensão AC')

# Marcador 'X' vermelho para os eventos de blackout
scat_blackout, = ax1.plot([], [], marker='x', color='red', linestyle='None',
                            markersize=10, zorder=3, label='Evento: Blackout')

ax1.set_title("Monitoramento da Rede - Modo Osciloscópio com Alertas")
ax1.set_ylabel("Tensão (V)")
ax1.set_xlabel("Tempo Relativo na Janela (s)")
ax1.set_ylim(-350, 350)
ax1.set_xlim(0, JANELA_SEGUNDOS) # Eixo X fixo de 0 a 0.1s
ax1.grid(True, linestyle=':', alpha=0.5)
ax1.legend(loc='upper left')

def update(_):
    global serial_buffer
    dados_novos = False

    ###############################################
    ############ LEITURA REAL DA UART #############
    ###############################################
    if ser and ser.in_waiting > 0:
        serial_buffer.extend(ser.read(ser.in_waiting))

        while len(serial_buffer) >= TAMANHO_PACOTE_TOTAL:
            idx = serial_buffer.find(TAG)
            if idx == -1:
                if serial_buffer[-1] == ord('S'):
                    serial_buffer = bytearray([ord('S')])
                else:
                    serial_buffer.clear()
                break

            if idx + TAMANHO_PACOTE_TOTAL <= len(serial_buffer):
                pacote_dados = serial_buffer[idx+2 : idx+8]
                try:
                    v, b, s = struct.unpack(PACOTE, pacote_dados)
                    # Guarda a tensão junto com o estado de blackout e surto
                    historico_dados.append((v, b, s))
                    dados_novos = True
                except struct.error:
                    pass
                serial_buffer = serial_buffer[idx + TAMANHO_PACOTE_TOTAL:]
            else:
                serial_buffer = serial_buffer[idx:]
                break

    ###############################################
    ######### ALGORITMO DE TRIGGER (GATILHO) ######
    ###############################################
    if dados_novos and len(historico_dados) >= PONTOS_TELA:
        lista_dados = list(historico_dados)
        idx_trigger = -1
        
        # Procura o ponto onde a onda cruza o zero subindo (baseado no valor de tensão d[0])
        for i in range(len(lista_dados) - PONTOS_TELA - 1, 0, -1):
            if lista_dados[i][0] <= 0 and lista_dados[i+1][0] > 0:
                idx_trigger = i
                break
        
        # Recorta a janela sincronizada pelo trigger
        if idx_trigger != -1:
            janela = lista_dados[idx_trigger : idx_trigger + PONTOS_TELA]
        else:
            janela = lista_dados[-PONTOS_TELA:]

        # Separa os vetores de tensão e blackout da janela recortada
        tensao_janela = [d[0] for d in janela]
        blackout_janela = [d[1] for d in janela]
        
        # Base de tempo estática local de 0 a 0.1 segundos
        tempos_janela = [i / TAXA_ENVIO for i in range(len(tensao_janela))]
        
        # 1. Atualiza o traçado da curva senoidal
        line1.set_data(tempos_janela, tensao_janela)

        # 2. Filtra e extrai apenas as coordenadas onde ocorreu Blackout (b == 1)
        tempos_blackout = []
        valores_blackout = []
        for i in range(len(janela)):
            if blackout_janela[i] == 1:
                tempos_blackout.append(tempos_janela[i])
                valores_blackout.append(tensao_janela[i])
        
        # Atualiza a marcação dos "X" vermelhos no gráfico
        scat_blackout.set_data(tempos_blackout, valores_blackout)

    return line1, scat_blackout

# Adicionado scat_blackout no retorno para o blit=True atualizar ambos os elementos de forma veloz
ani = FuncAnimation(fig, update, interval=30, blit=True)
plt.tight_layout()
plt.show()
