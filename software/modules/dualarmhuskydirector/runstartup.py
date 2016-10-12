sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/modules'))

import dualarmhuskydirector.startup
dualarmhuskydirector.startup.startup(robotSystem, globals())
