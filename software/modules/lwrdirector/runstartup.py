sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/modules'))

import lwrdirector.startup
lwrdirector.startup.startup(robotSystem, globals())
