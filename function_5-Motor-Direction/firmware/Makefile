
FRAMAC_OPT = -machdep x86_32 -eva-context-valid-pointers -cpp-command "gcc -E -C -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/ -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ -IDrivers/CMSIS/Include/ -DUSE_FULL_LL_DRIVE -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ -DSTM32F103xB -m32"

GCC_OPT := -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/ -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ -IDrivers/CMSIS/Include/ -DUSE_FULL_LL_DRIVE -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ -DSTM32F103xB

PCGE_DIR = $(HOME)/Tools/PCGE

SRC_C = $(wildcard Src/*.c Drivers/STM32F1xx_HAL_Driver/Src/*.c)

CMOCKA_PATH=$(HOME)/Outils/cmocka-1.1.5/install/

export LD_LIBRARY_PATH=$(CMOCKA_PATH)/lib

# commande pour tester les options de compilation
gcc	:
	gcc -c Src/steering.c $(GCC_OPT)

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

all_dot	:
	frama-c -load-script $(PCGE_DIR)/plug.ml $(FRAMAC_OPT)  Src/*.c -pcg-head -pcg-all

LST_DOT	=	$(wildcard *.dot)
LST_SVG =	$(LST_DOT:%.dot=%.svg)

all_svg	:	$(LST_SVG)

%.svg	:	%.dot
	dot -Tsvg -o $@ $+

metrics	:

	frama-c -load-script $(PCGE_DIR)/plug.ml -keep-comments $(FRAMAC_OPT)  $(SRC_C) -pcg-comment -pcg-comments f1.csv

########## CMOCKA ###################

%.o	:	%.c
	gcc -c -g -Wl,--wrap=steering_set_speed $(GCC_OPT) -I$(CMOCKA_PATH)/include -L$(CMOCKA_PATH)/lib -lcmocka $+ -o $@

cm_test1	:	Src/steering.o Tests/tests_steering__1.o
	gcc -Wl,--wrap=steering_set_speed $+ -o $@ -L$(CMOCKA_PATH)/lib -lcmocka
	./$@

clean_files	:=	$(wildcard *.svg *.dot *.o cm_test1 */*.o)
clean	:
	\rm -f $(clean_files)
