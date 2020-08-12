
all:
	make -C ./Fusion $(MAKECMDGOALS)
	make -C ./icm20948driver $(MAKECMDGOALS)
	make -C ./icmtest $(MAKECMDGOALS)

