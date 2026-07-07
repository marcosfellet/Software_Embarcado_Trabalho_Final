import serial
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import serial.tools.list_ports
import time
import struct

# --- CONFIGURAÇÕES ---
SIMULACAO = True
BAUD_RATE = 921600
JANELA_SEGUNDOS = 5.0 

# --- MAPEAMENTO DO PROTOCOLO ---
CABECALHO = b'SG'
# Tamanho do pacote: <fbb = 1 float (4 bytes) + 2 chars/bytes (2 bytes) = 6 bytes
FORMATO_PAYLOAD = "<fbb" 
TAMANHO_PAYLOAD = struct.calcsize(FORMATO_PAYLOAD) # 6 bytes
TAMANHO_PACOTE_TOTAL = len(CABECALHO) + TAMANHO_PAYLOAD # 8 bytes



def encontrar_porta_esp32():
    '''
    Função para detectar e selecionar a porta utilizada pelo ESP32
    '''
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
    ser = serial.Serial(porta, BAUD_RATE, timeout=0.05)

# --- CONTROLE DE TEMPO E MEMÓRIA ---
tempo_inicio = None 

# Um buffer global para acumular bytes bagunçados antes de decodificar
serial_buffer = bytearray()
contador_de_eventos = 0
# Criação de deques para armazenar os valores pontos que aparecerão no gráfico
tempo_data = deque()
tensao_data = deque()

blackout_events = deque() 

#########################################################################################
########################## Configuração visual do Matplotlib ############################ 
#########################################################################################

fig, ax1 = plt.subplots(figsize=(10, 6))

line1, = ax1.plot([], [], color='turquoise', lw=1.5, label='Tensão Nominal')

scat_blackout = ax1.plot([], [], marker='x', color='red', linestyle='None', 
                         markersize=10, zorder=3, label='Evento: Blackout')[0]

ax1.set_title("Monitoramento da Rede")
ax1.set_ylabel("Tensão (V)")
ax1.set_xlabel("Tempo de Medição (segundos)")

ax1.set_ylim(0, 350)
ax1.legend(loc='upper left', shadow=False, frameon=True)


def update(_):
    '''
    Função que atualiza o gráfico dinâmico com as variáveis recebidas via UART
    '''
    global contador_de_eventos, serial_buffer, tempo_inicio
    
    dados_novos = False

    # Checa se o usuário deseja fazer uma simulação para ver o comportamento do código ou não
    if SIMULACAO:
        if tempo_inicio is None:
            tempo_inicio = time.time()
            
        tempo_atual = time.time() - tempo_inicio 
        v = random.uniform(210, 230)
        
        b = 1 if random.random() > 0.95 else 0
        
        tempo_data.append(tempo_atual)
        tensao_data.append(v)
        if b: blackout_events.append((tempo_atual, v))
        
        dados_novos = True
        
    else:
        # Puxa tudo o que estiver no cabo USB e joga no buffer
        if ser and ser.in_waiting > 0:
            serial_buffer.extend(ser.read(ser.in_waiting))
            
            # Varre o buffer até processar todos os pacotes válidos que chegarem
            while len(serial_buffer) >= TAMANHO_PACOTE_TOTAL:
                # Procura a tag 'SG' no meio dos textos soltos
                idx = serial_buffer.find(CABECALHO)
                
                if idx == -1:
                    # Se não achou 'SG', limpa o lixo (preserva o último byte caso seja 'S')
                    if serial_buffer[-1] == ord('S'):
                        serial_buffer = bytearray([ord('S')])
                    else:
                        serial_buffer.clear()
                    break
                    
                if idx + TAMANHO_PACOTE_TOTAL <= len(serial_buffer):
                    # Achou 'SG' e tem bytes suficientes, extrai os 6 bytes seguintes do payload
                    pacote_dados = serial_buffer[idx+2 : idx+8]
                    try:
                        # O unpack dos dados (desempacota o 's' para limpar o buffer, mas não usa no gráfico)
                        v, b, s = struct.unpack(FORMATO_PAYLOAD, pacote_dados)
                            
                        contador_de_eventos += 1
                        
                        tempo_data.append(contador_de_eventos)
                        tensao_data.append(v)
                        
                        # Registra coordenadas dos blackouts
                        if b: blackout_events.append((contador_de_eventos, v))
                        
                        dados_novos = True
                    except struct.error:
                        pass
                        
                    # Recorta o buffer, deletando o pacote que acabamos de ler
                    serial_buffer = serial_buffer[idx + TAMANHO_PACOTE_TOTAL:]
                else:
                    # Achou o 'SG', mas o pacote está incompleto (faltam pedaços).
                    # Deleta o lixo que veio antes do 'SG' e aguarda a próxima leitura.
                    serial_buffer = serial_buffer[idx:]
                    break

    # Se novos dados entraram nas filas (seja da simulação ou real), atualiza a tela
    if dados_novos:
        
        while tempo_data and tempo_data[0] < limite_esquerdo:
            # Obtém e remove o elemento do deque 
            tempo_data.popleft()
            tensao_data.popleft()
            
        while blackout_events and blackout_events[0][0] < limite_esquerdo: blackout_events.popleft()

        # Adiciona o ponto ao gráfico
        line1.set_data(list(tempo_data), list(tensao_data))
        
        if blackout_events:
            tempos, valores = zip(*blackout_events)
            scat_blackout.set_data(tempos, valores)
        else:
            scat_blackout.set_data([], [])


    return line1, scat_blackout

ani = FuncAnimation(fig, update, interval=30, blit=False)
plt.tight_layout()
plt.show()
