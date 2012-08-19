This is the board support integration branch for imageproc-lib on the
ImageProc 2.5. Please send pull requests with ImageProc 2.5-related changes
to this branch.

Use the following commands to setup the integration branch in your local git 
repository. These commands assume you have forked imageproc-lib from 
biomimetics into your github account, and then cloned your github account into
your local repository.

In Git Bash:
```bash
git remote add -t imageproc2.5-integration imageproc2.5-integration git@github.com:ryanjulian/imageproc-lib.git
git fetch imageproc2.5-integration
git checkout -b imageproc2.5-integration -t imageproc2.5-integration/imageproc2.5-integration
git push origin imageproc2.5-integration
git remote set-url --push imageproc2.5-integration `git config remote.origin.url`
```
=============================================================================

All code licensed under the 3-clause modified-BSD license, unless
otherwise noted.

=====================================
Shared library for ImageProc projects
=====================================

Description:
 A set of common modules developed for the ImageProc family of boards,
 which use Microchip's dsPIC architecture.

Download code:
 https://github.com/biomimetics/imageproc-lib

Usage:
 Place this library under the embedded source directory where your
 ImageProc project resides (e.g. embedded/imageproc-lib/ if your project
 is in embedded/project/), then modify your project so that it knows
 where to find the library, and finally compile.

Citing the library:
 If you would like to reference this code in a publication, please cite
 these conference papers:

 - | Fernando L. Garcia Bermudez and Ronald S. Fearing (2009)
   | `Optical Flow on a Flapping Wing Robot
     <http://dx.doi.org/10.1109/IROS.2009.5354337>`_
   | In IROS 2009. St. Louis, MO, USA.

 - | Stanley S. Baek, Fernando L. Garcia Bermudez, and Ronald S. Fearing (2011)
   | `Flight Control for Target Seeking by 13 gram Ornithopter
     <http://dx.doi.org/10.1109/IROS.2011.6094581>`_
   | In IROS 2011. San Francisco, CA, USA.

 This material is based upon work supported by the National Science
 Foundation under Grant No. IIS-0705429 and IIS-0931463. Any opinions,
 findings, and conclusions or recommendations expressed in this material
 are those of the author(s) and do not necessarily reflect the views of
 the National Science Foundation.
