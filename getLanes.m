im = imread('test.jpg');
[l1,x,y] = roipoly(im);

lane3.mask = l1;
lane3.poly= [x,y];