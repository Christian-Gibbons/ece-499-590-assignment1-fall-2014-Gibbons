def robotChecksum(commandPacket):
	checksum = 0
	length = commandPacket[3]
	for i in range (2, 3+length):
		checksum += commandPacket[i]
	checksum = ~checksum
	checksum = checksum & 0xFF
	commandPacket[3+length] = checksum
	return commandPacket

