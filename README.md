utvisioncomponents
===============
This is the utvisioncomponents Ubitrack submodule.

Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the utvisioncomponents by executing:

    git submodule add https://github.com/schwoere/utvisioncomponents.git modules/utvisioncomponents

Description
----------
The utvisioncomponents contains components working on camera images (based on utvision methods). Also contains components to capture and transfer images and videos.


Dependencies
----------
In addition, this module has to following submodule dependencies which have to be added for successful building:

<table>
  <tr>
    <th>Dependency</th><th>Dependent Components</th><th>optional Dependency</th>
  </tr>
  <tr>
    <td>utdataflow, utvision</td><td>utVisionComponents, utVisionIOComponents</td><td>no</td>
  </tr>
  <tr>
    <td>utcomponents</td><td>ImageTrigger, FrameBuffer, ImageGate, FrameSampler</td><td>yes</td>
  </tr>
</table>
