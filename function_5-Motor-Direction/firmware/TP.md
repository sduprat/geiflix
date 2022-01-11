# Prerequisites

* place into the working directory `geiflix/function_5-Motor-Direction/firmware`
* install external plugin (PCGE - [https://github.com/sduprat/PCGE](https://github.com/sduprat/PCGE))
``` shell
make install_pcge
```

# Use additional plugin features
Plugin PCGE is an external plugin called with the option `-load-script $(PCGE_DIR)/plug.ml` in the Makefile.

## Graphs
Generate module dependencies graph and function call graph :
``` shell
make graphs
firefox *.svg
```
`SRC_C` variable in the `Makefile` can be modified to select source files

<em>Present graphs of your project</em>

other : one graph per module :
```shell
make all_dot
make all_svg
```

## Metrics
Generate metrics per function (nb locs, nb comments, max depth in structure control)
``` shell
make metrics
```
This generate a .csv file.

<em>Present metrics of your project.</em>

# Value analysis
## Recalls
Value analysis is the Frama-C feature provided by the <em>Eva</em> plugin and used to detect runtime errors.
A Frama-C Value analysis is triggered by the `-eva` option.
Other options are activated (see FRAMAC_OPT in the Makefile.

A generic Makefile rule `eva_%` is set to run analysis on each specific finction.  
To make an analysis starting at the steering_Init() function, launch :
```shell
make eva_steering_Init  
```
To make an analysis starting at the main() function, launch :
```shell
make eva_main  
```
## Launch analysis on steering_Init()

Launch analysis and explain `eva` error messages.
Add contracts (requires and ensures) to limits ranges of in put and output values for different functions.

<em>Relaunch analysis, explain.</em>

Add a function contract to `steering_get_angle` with an ensure to express that valeus of ADCBUF are on 12 bits (see comments in main.c`).

<em>Relaunch analysis, explain.</em>

## Launch analysis on main()

<em>Relaunch analysis, explain.</em>

<em>See ranges of output variables at the end of the console, check if values are correct (with functional, software, hardawre requirements) </em>

## Conclusion
<em>In your opinion, what can be the benefits of this verification regarding global objectives of Safety ? </em>

## Bonus (unit tests)
To launch unit test :
```shell
make cm_test1
```
To do:
- study logic of mock and wrap mocked functions
- complete tests





