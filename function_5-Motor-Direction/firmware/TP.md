# Prerequisites

* place into the workig directory `geiflix/function_5-Motor-Direction/firmware`
* install external plugin (PCGE)
``` shell
make install_pcge
```

# Use additional plugin features
* generate module dependencies graph and function call graph :
``` shell
make graph
firefox *.svg
```
`SRC_C` variable in the `Makefile` can be modified to select source files

<em>Present graphs of your project</em>

generate metrics per function (nb locs, nb comments, mas depth in structure control)
``` shell
make metrics
```
This generate a .csv file.

<em>Present metrics of your project.</em>



