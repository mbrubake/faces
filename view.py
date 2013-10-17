#! /usr/bin/env python
import sys
from tvtk.api import tvtk
from mayavi import mlab


filename = sys.argv[1]
stlr = tvtk.STLReader()
stlr.file_name = file_name = filename
stlr.update()
stld = stlr.output

fig = mlab.figure(bgcolor=(1,1,1), fgcolor=(0,0,0))
surf = mlab.pipeline.surface(stld, opacity=1, color=(0,1,1))
edge = mlab.pipeline.extract_edges(surf)
edge_surf = mlab.pipeline.surface(edge,opacity=.1, color=(1,0,0))
mlab.show()
