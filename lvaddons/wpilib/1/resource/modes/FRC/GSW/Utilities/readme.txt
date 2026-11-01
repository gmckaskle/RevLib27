This Utilities folder allows you to customize which utilities appear on the Utilities page of the LabVIEW for FRC Getting Started Window.

There are two files that you create in order to have your utility application appear. Create an xml file to define the utility name and launch parameters.  Then, create a graphics file (png, jpg, or bmp) for the utlity glyph.

The xml file can be any name, but must use the .xml extension.  Open one of the existing xml files to view the format.  Use the following parameters to define your utlity:

<Order>number</Order> optional : determines the top down list order of the utilities : number is an integer : numbers 1, 2, and 3 are reserved for National Instruments

<Name>name of your utility</Name> required

<LabVIEWRelativePath>True or False</LabVIEWRelativePath> optional

<32bitRelativePath>True or False</32bitRelativePath> optional

<64bitRelativePath>True or False</64bitRelativePath> optional

<PublicDocsRelativePath>True or False</PublicDocsRelativePath> optional

<Path>path to your exe</Path> required : the full path to your exe unless one of the relative path parameters is set to True

<PassParameters>parameters to pass to your exe</PassParameters> optional

<GraphicsType>png or jpg or bmp</GraphicsType> required if you provide a graphics file



The graphics file is optional and may be a png, jpg, or bmp file.  It should be 16 x 16 pixels in size.  A generic Windows graphic will be used if you do not provide a file.