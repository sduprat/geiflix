
FRAMAC_OPT = -machdep x86_32 -eva-context-valid-pointers -cpp-command "gcc -E -C -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/ -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ -IDrivers/CMSIS/Include/ -DUSE_FULL_LL_DRIVE -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ -DSTM32F103xB -m32"

PCGE_DIR = ~/Tools/PCGE

SRC_C = $(wildcard Src/*.c Drivers/STM32F1xx_HAL_Driver/Src/*.c)

# commande pour tester les options de compilation
gcc	:
	gcc -c Src/steering.c -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/ -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ -IDrivers/CMSIS/Include/ -DUSE_FULL_LL_DRIVE -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ -DSTM32F100xB

#clone les scripts pcge dand $(PCGE_DIR)
install_pcge	:
	mkdir -p $(PCGE_DIR)
	git clone https://github.com/sduprat/PCGE.git $(PCGE_DIR)

# lancement de l'analyse statique sur tous les sources et analyse à partir du point d'entrée % dans main_%
eva_%	:
	frama-c-gui -eva-precision 8 -lib-entry -eva -absolute-valid-range 0-0xFFFFFFFF -main $* $(FRAMAC_OPT) $(SRC_C)

# generation des graphes de dépendance de modules et de fonction
graphs	:
	frama-c -load-script $(PCGE_DIR)/plug.ml $(FRAMAC_OPT)  $(SRC_C) -pcg-m m.dot -pcg-f f.dot
	dot -Tsvg -o m.svg m.dot
	dot -Tsvg -o f.svg f.dot

metrics	:
	frama-c -load-script $(PCGE_DIR)/plug.ml -keep-comments $(FRAMAC_OPT)  $(SRC_C) -pcg-comment -pcg-comments f1.csv
