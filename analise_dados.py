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
BAUD_RATE = 115200
JANELA_SEGUNDOS = 5.0 

# --- MAPEAMENTO DO PROTOCOLO ---
CABECALHO = b'SG'
FORMATO_PAYLOAD = "<ffbb" # Apenas os dados sem o cabeçalho
TAMANHO_PAYLOAD = struct.calcsize(FORMATO_PAYLOAD) # 10 bytes
TAMANHO_PACOTE_TOTAL = len(CABECALHO) + TAMANHO_PAYLOAD # 12 bytes

def encontrar_porta_esp32():
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

# Novo: Um buffer global para acumular bytes bagunçados antes de decodificar
serial_buffer = bytearray()

tempo_data = deque()
tensao_data = deque()
corrente_data = deque()

blackout_events = deque() 
surtos_events = deque()

# Configuração visual do Matplotlib
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

line1, = ax1.plot([], [], color='turquoise', lw=1.5, label='Tensão Nominal')
line2, = ax2.plot([], [], 'r-', lw=1.5, label='Corrente Nominal')

scat_blackout = ax1.plot([], [], marker='x', color='red', linestyle='None', 
                         markersize=10, zorder=3, label='Evento: Blackout')[0]

scat_surtos = ax1.plot([], [], marker='o', color='darkslategrey', linestyle='None', 
                       markersize=6, zorder=3, label='Evento: Surto')[0]

ax1.set_title("Monitoramento da Rede (Leitura de Estrutura de Bytes C)")
ax1.set_ylabel("Tensão (V)")
ax2.set_ylabel("Corrente (A)")
ax2.set_xlabel("Tempo de Medição (segundos)")

ax1.set_ylim(100, 300)
ax2.set_ylim(0, 100)

ax1.legend(loc='upper left', shadow=False, frameon=True)
ax2.legend(loc='upper left', shadow=False, frameon=True)

def update(_):
    global tempo_inicio, serial_buffer
    
    dados_novos = False
    
    if SIMULACAO:
        if tempo_inicio is None:
            tempo_inicio = time.time()
            
        tempo_atual = time.time() - tempo_inicio 
        v = random.uniform(210, 230)
        i = random.uniform(10, 20)
        b = 1 if random.random() > 0.95 else 0
        s = 1 if random.random() > 0.95 else 0
        
        tempo_data.append(tempo_atual)
        tensao_data.append(v)
        corrente_data.append(i)
        if b: blackout_events.append((tempo_atual, v))
        if s: surtos_events.append((tempo_atual, v))
        
        dados_novos = True
        
    else:
        # Puxa tudo o que estiver no cabo USB e joga no buffer
        if ser and ser.in_waiting > 0:
            serial_buffer.extend(ser.read(ser.in_waiting))
            
            # Varre o buffer até processar todos os pacotes válidos que chegarem
            while len(serial_buffer) >= TAMANHO_PACOTE_TOTAL:
                # Procura a assinatura 'SG' no meio dos textos soltos
                idx = serial_buffer.find(CABECALHO)
                
                if idx == -1:
                    # Se não achou 'SG', limpa o lixo (preserva o último byte caso seja 'S')
                    if serial_buffer[-1] == ord('S'):
                        serial_buffer = bytearray([ord('S')])
                    else:
                        serial_buffer.clear()
                    break
                    
                if idx + TAMANHO_PACOTE_TOTAL <= len(serial_buffer):
                    # Achou 'SG' e tem bytes suficientes! Extrai os 10 bytes seguintes.
                    pacote_dados = serial_buffer[idx+2 : idx+12]
                    
                    try:
                        v, i, b, s = struct.unpack(FORMATO_PAYLOAD, pacote_dados)
                        
                        if tempo_inicio is None:
                            tempo_inicio = time.time()
                            
                        tempo_atual = time.time() - tempo_inicio 
                        
                        tempo_data.append(tempo_atual)
                        tensao_data.append(v)
                        corrente_data.append(i)
                        if b: blackout_events.append((tempo_atual, v))
                        if s: surtos_events.append((tempo_atual, v))
                        
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
        # A referência de tempo passa a ser o último dado que entrou na fila
        tempo_referencia = tempo_data[-1]
        limite_esquerdo = tempo_referencia - JANELA_SEGUNDOS
        
        while tempo_data and tempo_data[0] < limite_esquerdo:
            tempo_data.popleft()
            tensao_data.popleft()
            corrente_data.popleft()
            
        while blackout_events and blackout_events[0][0] < limite_esquerdo: blackout_events.popleft()
        while surtos_events and surtos_events[0][0] < limite_esquerdo: surtos_events.popleft()

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

        if tempo_referencia < JANELA_SEGUNDOS:
            ax1.set_xlim(0, JANELA_SEGUNDOS)
            ax2.set_xlim(0, JANELA_SEGUNDOS)
        else:
            ax1.set_xlim(limite_esquerdo, tempo_referencia)
            ax2.set_xlim(limite_esquerdo, tempo_referencia)

    return line1, line2, scat_blackout, scat_surtos

ani = FuncAnimation(fig, update, interval=30, blit=False)
plt.tight_layout()
plt.show()
