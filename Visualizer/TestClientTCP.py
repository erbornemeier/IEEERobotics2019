import socket

TCP_IP = '138.67.207.188'
TCP_PORT = 4445

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

path = [(88, 36, -45), (88, 8, -90), (76, 9, 0)]
msg = "draw-path {}\r\n".format(path)
print("Sending: \"{}\"".format(msg))
s.send(bytearray(msg, "utf-8"))
#s.send(b"draw-path [(88, 36, -45), (88, 8, -90), (76.59782335945356, 9.70085346410806, 0)]\r\n")
s.close()
