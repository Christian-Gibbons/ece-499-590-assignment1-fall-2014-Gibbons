from robot_checksum import robotChecksum
def createCommandPacket(ID, length, instruction, parameters):
	commandPacket = [0xFF, 0xFF, ID, length, instruction] + parameters + [0]
	commandPacket = robotChecksum(commandPacket)
	return commandPacket

