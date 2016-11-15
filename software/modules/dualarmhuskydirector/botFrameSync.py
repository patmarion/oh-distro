#!/usr/bin/env python
"""
botFrameSync

FILE
    /oh-distro/software/modules/dualarmhuskydirector/botFrameSync.py

DESCRIPTION
    Contains a method called createBoxAffordanceAndSyncToBotFrame to sync a box affordance with a 
    given bot frame. See function help for details.

METHODS
    createBoxAffordanceAndSyncToBotFrame
    createGraspFrameAboveBox
   
"""
### IMPORTS ###

import numpy as np

from director import transformUtils # Used to instantiate new frames and transform frames
from director import objectmodel as om # Interact with objects in Model Browser (left pannel in director)

def createBoxAffordanceAndSyncToBotFrame(getBotFrame,affordancePanel,playbackRobotModel,vis):
    """
    createBoxAffordanceAndSyncToBotFrame

    SYNTAX
        frameSync = createBoxAffordanceAndSyncToBotFrame(getBotFrame,affordancePanel,playbackRobotModel,vis)

    DESCRIPTION
        Creates a box affordance whose central frame is synchronized with the bot frame called
        botFrameTransformName (see parameters section).

    PARAMETERS
        getBotFrame (function handle) : Used to aquire bot frame in function scope.
        affordancePanel (class) : Class to interact with affordances.
        playbackRobotModel (class) : Class to interact with robot model.
        vis (class) : Class to interact with director visualization.

    RETURNS
        frameSync (class) : Frame sync object.

    NOTE
        No thought is needed for passing in parameters, simply copy-and-paste interface into 
        Python console.

    """

    # Initialise bot frame in local scope
    botFrameTransformName = 'RUBBISH_BIN'
    botFrameTransform = getBotFrame(botFrameTransformName)
    bot = om.findObjectByName(botFrameTransformName)
    botFrame = bot.getChildFrame()

    # Spawn slave box affordance
    box = affordancePanel.onSpawnBox()
    boxFrame = box.getChildFrame()
    boxFrameTransform = boxFrame.transform
    box.setProperty('Dimensions', (0.46, 0.56, 0.46))

    # Send box to world origin
    boxFrameTransform.Translate( -np.array(boxFrameTransform.GetPosition()) )

    # Send box to desired location
    ## Form desired position in bot ptB = [0,0,0.23]
    ptB = np.zeros((4,1))
    ptB[2,0] = 0.23
    ptB[3,0] = 1.0 
    
    ## Get transformation matrix for bot frame
    T = transformUtils.getNumpyFromTransform(botFrameTransform)
    
    ## Compute desired position of box centre in world frame
    ptW = np.dot(T, ptB)
    
    ## Translate box to new position 
    boxFrameTransform.Translate(ptW[:3])

    # Orient box frame
    botMatrix = botFrameTransform.GetMatrix()
    boxMatrix = boxFrameTransform.GetMatrix()
    for i in range(3):
        for j in range(3):
            boxMatrix.SetElement(i,j,botMatrix.GetElement(i,j))

    # Sync bot and box frames
    frameSync = vis.FrameSync()
    frameSync.addFrame(om.findObjectByName(botFrameTransformName))
    frameSync.addFrame(om.findObjectByName('box frame'))

    return frameSync, boxFrame
    
def createGraspFrameAboveBox(boxFrame,vis):
    """
    """
    # Get box transform
    boxFrameTransform = boxFrame.transform

    # Spawn new frame and transform
    ## Spawn at world origin
    graspFrameTransform = transformUtils.frameFromPositionAndRPY([0]*3,[0]*3)
    vis.showFrame(graspFrameTransform, 'grasp frame')
    
    ## Transform to box position and orientation
    ### Form desired point in box frame
    ptB = np.zeros((4,1)); ptB[1,0] = -0.69; ptB[3] = 1

    ### Get box transformation
    T = transformUtils.getNumpyFromTransform(boxFrameTransform)
    
    ### Get desired point in world frame
    ptW = np.dot(T, ptB)

    ### Get orientation and rotate
    theta = np.sign(ptW[1])*np.arctan(abs(ptW[1])/abs(ptW[0]))
    graspFrameTransform.RotateZ(np.rad2deg(theta))

    ### Translate grasp frame
    graspFrameTransform.Translate(ptW[:3])

    return graspFrameTransform
