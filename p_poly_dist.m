%*******************************************************************************
% function:	p_poly_dist
% Description:	distance from point to polygon whose vertices are specified by the
%              vectors xv and yv
% Input:  
%    x - point's x coordinate
%    y - point's y coordinate
%    xv - vector of polygon vertices x coordinates
%    yv - vector of polygon vertices x coordinates
% Output: 
%    d - distance from point to polygon (defined as a minimal distance from 
%        point to any of polygon's ribs, positive if the point is outside the
%        polygon and negative otherwise)
%    x_poly: x coordinate of the point in the polygon closest to x,y
%    y_poly: y coordinate of the point in the polygon closest to x,y
%
% Routines: p_poly_dist.m
% Revision history:
%    03/31/2008 - return the point of the polygon closest to x,y
%               - added the test for the case where a polygon rib is 
%                 either horizontal or vertical. From Eric Schmitz.
%               - Changes by Alejandro Weinstein
%    7/9/2006  - case when all projections are outside of polygon ribs
%    23/5/2004 - created by Michael Yoshpe 
% Remarks:
%*******************************************************************************
function [d,x_poly,y_poly] = p_poly_dist(x, y, xv, yv) 

% If (xv,yv) is not closed, close it.
xv = xv(:);
yv = yv(:);
Nv = length(xv);
if ((xv(1) ~= xv(Nv)) || (yv(1) ~= yv(Nv)))
    xv = [xv ; xv(1)];
    yv = [yv ; yv(1)];
%     Nv = Nv + 1;
end

% linear parameters of segments that connect the vertices