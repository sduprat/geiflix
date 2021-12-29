# Prerequisites

* place into the workig directory `geiflix/function_5-Motor-Direction/firmware`
* install external plugin (PCGE)
``` shell
make install_pcge
```

# Use additional plugin features
Plugin PCGE is an external plugin called with the option `-load-script $(PCGE_DIR)/plug.ml` in the Makefile.

## Graphs
Generate module dependencies graph and function call graph :
``` shell
make graph
firefox *.svg
```
`SRC_C` variable in the `Makefile` can be modified to select source files

<em>Present graphs of your project</em>

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
Add contracts (requires and ensures) to limits ranges of values.





