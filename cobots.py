import socket
import binascii
import PySimpleGUI as sg
import struct

# Crear la interfaz gráfica de usuario con PySimpleGUI
sg.theme('DefaultNoMoreNagging')
layout = [
    [sg.Text('Dirección IP del dispositivo:'), sg.InputText(key='ip')],
    [sg.Text('Puerto de acceso:'), sg.InputText(key='port')],
    [sg.Text('Cadena hexadecimal a enviar:'), sg.InputText(key='data')],
    [sg.Button('Enviar datos'), sg.Button('Salir')]
]
window = sg.Window('Envío de datos Modbus TCP/IP', layout)

# Función para enviar datos a través de Modbus TCP/IP
def send_modbus_tcp(ip, port, data):
    try:
        # Crear un socket TCP/IP
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Conectar el socket al puerto donde el dispositivo está escuchando
        server_address = (ip, int(port))
        sock.connect(server_address)
        # Convertir la cadena hexadecimal a bytes
        hex_data = binascii.unhexlify(data)
        # Enviar los datos al dispositivo
        sock.sendall(hex_data)
        # Recibir la respuesta del dispositivo
        response = sock.recv(1024)
        # Imprimir la respuesta recibida
        print('Respuesta recibida:', binascii.hexlify(response))
    finally:
        # Cerrar el socket
        sock.close()

def deg_to_rad(deg):
    return deg * (3.14159265359 / 180)

def float_to_8bit_string(f):
    # Convert float to hexadecimal string
    hex_string = hex(struct.unpack('<I', struct.pack('<f', f))[0])[2:]
    # Ensure the hexadecimal string has 8 characters
    hex_string = hex_string.zfill(8)
    return hex_string

def code(array):
    string = "00010002002917"
    for i in array:
        string += float_to_8bit_string(i)
    string += "c2b8b23e58a00b4100000000"
    print(string)
    return string


# Bucle principal de la interfaz gráfica de usuario
while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED or event == 'Salir':
        break
    if event == 'Enviar datos':
        #ip = values['ip']
        #port = values['port']
        #data = values['data']
        # Enviar los datos a través de Modbus TCP/IP
        #send_modbus_tcp(ip, port, data)
        send_modbus_tcp("192.168.0.41", 502, '0001000200030b0801')

        send_modbus_tcp("192.168.0.41", 502, '00010002000d19db0f4940f366df4000000000')


        #send_modbus_tcp("192.168.0.41", 502, '00010002002917000000004222000042c3666642a7333342b90000c1f6666600000000c2b8b23e58a00b4100000000')
        #send_modbus_tcp("192.168.0.41", 502, '00010002002917920a36bf000000000000000000000000000000000000000000000000c2b8b23e58a00b4100000000')
        #send_modbus_tcp("192.168.0.41", 502, '0001000200291700000000000000000000000000000000000000000000000000000000c2b8b23e58a00b4100000000')
        #send_modbus_tcp("192.168.0.41", 502, '000100020029170000000000000000DB0FC93F00000000000000000000000000000000c2b8b23e58a00b4100000000')

        #send_modbus_tcp("192.168.0.11", 502, '0001000200411b0086c1410000484300004843db0f494000000000000000000086c1410000000000004843db0f49400000000000000000000048420000c8420000fa4400000000')
        #send_modbus_tcp("192.168.0.41", 502, '0001000200411b0086c14100004843000048430000344300000B4C2000000000086c141000048c3000048430000344300000B4C200000000000048420000c8420000fa4400000000')
        positions = [#[0,6.9,97.7,0.6,86.8,-1.9,0],
					#[0,40.5,97.7,-90.5,84.8,33.1,0],
					[0,0,15,0,0,0,0],
					#[0,58.9,68.2,-1.8,94.5,-4.9,0],
					#[0,56.1,70.2,-1.8,-79.1,1.8,0]
     				]
        for pos in positions:
            send_modbus_tcp("192.168.0.41", 502,code(pos))

        #{00}{86}{C1}{41}
        #3FC90FDB
        #send_modbus_tcp("192.168.1.165", 502, '00010002000D19DB0F4940F366DF4000000000')


# Cerrar
# Cerrar la ventana de la interfaz gráfica de usuario
window.close()