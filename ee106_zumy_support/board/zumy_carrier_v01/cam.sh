#! /bin/bash
# andrew@rocketnumbernine.com - Use freely, for whatever, at your own risk.

# Modified by Andrew Pullin for Zumy v0.1 rev D board CAM

EAGLE=eagle

if [ $# -lt 1 ] ; then
        echo "usage $0 <path-to-board> [output-dir-and-file-prefix]"
        echo " e.g:  $0 my_boards/boardx.brd gerbers/board"
        echo "  will create gerber files with eagle in a directory called gerbers"
        echo "  and bundle into gerbers.zip"
        exit 1
fi

board="$1"
outputfile=${2:-"$(dirname $board)/$(basename $board .brd)"}
outputdir=$(dirname ${outputfile})

if [ ! -d ${outputdir} ]; then
        mkdir -p ${outputdir}
fi
set -e

### Specific CAM for ZUMY board below!
##  bDocu layer is added for art silkscreening on bottom of board
##  tDocu layer is added due to mbed part labelling (todo: fix in zumy.lbr)

# Board outline
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.bor ${board} Dimension   # unchanged

# top copper (cmp)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.cmp ${board} Top Pads Vias   # unchanged

# bottom copper (sol)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.sol ${board} Bottom Pads Vias   # unchanged

# top solder mask (stc)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.stc ${board} tStop   # unchanged

# bottom solder mask (sts)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.sts ${board} bStop   # unchanged

# top silkscreen (plc)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.plc ${board} tPlace tNames tValues tDocu  ##### added tDocu

# bottom silkscreen (pls)
${EAGLE} -X -dGERBER_RS274X -o${outputfile}.pls ${board} bPlace bNames bValues bDocu  ##### added bDocu

# create drill files
${EAGLE} -X -dEXCELLON -o${outputfile}.txt ${board} Drills Holes   # unchanged

# get drills used from dri file and create drl rack file 
# (like running drillcfg.ulp: ${EAGLE} -N- -C'RUN drillcfg.ulp; QUIT;' -o${outputfile} ${board}
#cat ${outputfile}.dri | sed -e 's/ *\(T[0-9][0-9]\) *\([0-9.]*..\).*/\1 \2/' | grep -e "^T[0-9][0-9]" > ${outputfile}.drl

#zip -r ${outputdir}.zip ${outputdir}
zip -j ${outputdir}.zip ${outputdir}/*.*
