using SerialPorts

sensor_1 = SerialPort("/dev/ttyUSB0", 9600)
sensor_2 = SerialPort("/dev/ttyUSB1", 9600)

i = 0
while i < 50
	#println("Sensor 1:", readavailable(sensor_1))
	#println("Sensor 2:", readavailable(sensor_2))
	x = readavailable(sensor_1)
	if x != ""
		println("Sensor 1 ", x)
		i += 1
		x = ""
	end
	y = readavailable(sensor_2)
	if y != ""
		println("Sensor 2 ", y)
		i += 1
		y = ""
	end
end