import socket

s = socket.socket()
host = socket.gethostname()
port = 3000

s.connect((host, port))

mess = str(float("9.3")/50)

#Need to add " ;" at the end so pd knows when you're finished writing.

message = mess + ";"
i = 0
while True:
	if i > 1400000:
		message = str(float("50.0")/50) + ";"
	s.send(message.encode('utf-8'))
	i += 1