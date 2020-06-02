taperratio = .8; //the reduced base thickness

translate([30,0,0])taperSnapArm(length = 10, lzoom = 1, cantWidth = 4, headL = 6, cantAngle = -10,headHeight = 4, mask = 0, d = 2, res = 3);
translate([60,0,0])taperSnapArm(length = 10, lzoom = 1, cantWidth = 4, headL = 6, cantAngle = -10,headHeight = 4, mask = 0.2, d = 2);
translate([-30,0,0])doubleTaperSnapArm(length = 10, lzoom = 1, cantWidth = 2, headL = 6, cantAngle = 0,headHeight = 4, mask = 0.0, d = 2,width = 10);
translate([0,0,0])beltSideSnap(length = 10, lzoom = 1, cantWidth = 2, headL = 6, cantAngle = 0,headHeight = 4, mask = 0.0, d = 2,width = 10);
translate([0,22,0])beltSideSnap(length = 10, lzoom = 1, cantWidth = 2, headL = 6, cantAngle = 0,headHeight = 4, mask = 0.2, d = 2,width = 10);

//translate([-30,0,0])doubleCantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0.0);
/*
   taken  from cantilver snap joints:
http://fab.cba.mit.edu/classes/S62.12/people/vernelle.noel/Plastic_Snap_fit_design.pdf
y= (permissible) deflection (=undercut)
E= (permissible) strain in the outer fiber at the root; 
1= length of arm
h= thickness at root 
b= width at root
c= distance between outer fiber and neutral fiber (center of gravity)
Z= section modulus Z = I c, where I = axial moment of inertia
Es= secant modulus (see Fig. 16) 
P= (permissible) deflection force
K= geometric factor (see Fig. 10
 */

//eps=0.5*20/100;//Elongation at break of PLA=6%
eps=0.5*6/100;//Elongation at break of PLA=6%
//straight cantilever Arm
function hForFixedArm(y,l, eps = eps)=0.67*eps*l*l*y;
function hForTaperArm(y,l, eps = eps)=1.09*eps*l*l/y;//means h at the end of the arm is .5 h from the start
function hForTaperXArm(y,l, eps = eps)=0.86*eps*l*l/y;//d goes from d to d/2
function maxFlexArm(b,h,E,l, eps = eps)= b*h*h/6*E*eps/l;
//trapezoidal cantilever arm
function hForFixedTrapez(a,b,y,l, eps = eps)=(a+b)/(2a+b)*eps*l*l/y;
function hForTaperTrapez(a,b,y,l, eps = eps)=1.64*(a+b)/(2a+b)*eps*l*l/y;
function hForTaperXTrapez(a,b,y,l, eps = eps)=1.28*(a+b)/(2a+b)*eps*l*l/y;
function maxFlexTrapez(b,h,E,l, eps = eps)= h*h*(a*a+4*a*b+b*b)*E*eps/(12*(2*a+b)*l);
//ring segment arm, K goes in dependence of theta from around 1 to 10
function yForFixedSeg(r,l,K=2, eps = eps)=K*eps*l*l/r;
function yForTaperSeg(r,l,K=2, eps = eps)=1.64*K*eps*l*l/r;
function yForTaperXSeg(r,l,K=2, eps = eps)=1.28*K*eps*l*l/r;
function maxFlexSeg(Z,E,l, eps = eps)= Z*E*eps/l;
//anyshape arm result is c meaning middle height depending of mass distribution usually h/2
function hForFixedIrreg(y,l, eps = eps)=1/3*eps*l*l/y;
function hForTaperIrreg(y,l, eps = eps)=0.55*eps*l*l/y;
function hForTaperXIrreg(y,l, eps = eps)=0.43*eps*l*l/y;
function maxFlexSeg(Z,E,l, eps = eps)= Z*E*eps/l;

/**
  taperArmPoly (l  arm length (without head)
  y  snap tooth width, 
  arm zoom (to make arm thicker, default  1 (no zoom), don't make it smaller, it will snap...
  angle of the head, to make snap permanent or easier reversible
  button  head height, to make ev a push button, 
  head total head heigth
  mask = value, >0 creates a mask forthe snap fit to be diffed with something, value added to shape
  eps material constant 2% usually, some say 6% for PLA, use 50% of snap force, so 3% should be safe 
  (default val)
 */
function cantCheckButton(b,h) = (b < 0)? 0  : (b > h)?h:b; 
  function taperArmPoly(l, y, lz = 1, angle, button  =0, head, mask = 0, eps =eps, smooth =0) = 
  let( h = lz*hForTaperArm(y,l, eps = eps))
let( hh = cantCheckButton(button,head))

  [ 
    (mask >  0)?[min(0,h-y-mask),0]                :[0,0], //0, start
    (mask >  0)?[h+y+mask,0]                       :[h,0], //1  
    (mask >  0)?[h+mask,l/2]                       :[h,l/2], //2 tapering for head insertion
    (mask >  0)?[h+mask,l-mask]                    :[h,l],   //3 head start
    (mask >  0)?[h+y+2*mask,l+y*tan(angle)-mask] :[h+y,l+y*tan(angle)], // 4 head cant low
  if(mask >  0) [h+y+2*mask,l+hh+mask]    
else if(smooth >0) each(roundPoint2D([h+y,l+hh],rad=y/10, res = 1*smooth,a0 = 20, a1 = 30, left = false)) 
  else                                              [h+y,l+hh], //5 head cant up
  //(mask >  0)?[h+y+mask,l+hh+mask]    :(smooth >0)? each(roundPoint2D([h+y,l+hh],rad=30, res = smooth,above = true, left = false)):[h+y,l+hh],
  (mask >  0)?[min(0,h-y-mask),l+head]             :[h/2,l+head], //6 top point
  (mask >  0)?[min(0,h-y-mask),l+head]             :[h/2,l], //7 head end
  ];
  function taperArmHeadPoly(l, y, lz = 1, angle, button  =0, head,  eps =eps, smooth =0, mask = 0) = 
  let( h = lz*hForTaperArm(y,l, eps = eps))
  let( hh = cantCheckButton(button,head))
  let( d = (mask == 0)?min(head/4, y/4): 0)
  [ 
    [h/2+d,l+d],
    [h+y-d,l+y*tan(angle)+d], // 4 head cant low
    if(smooth >0) each(roundPoint2D([h+y-d,l+hh-d],rad=y/10, res = 1*smooth,a0 = 20, a1 = 30, left = false)) 
  else [h+y-d,l+hh-d],
  [h/2+d,l+head-(y+h/2)*d/(y-d)],
  ];
function flatArray(a)= [
  if(len(a) > 2) each(a) else a];
/**
 beltSideSnap
 build a belt snapfit where the cantilevers are on the side, pinch the beltbucle to release it
 
 set mask >0 to build the bucle, = 0 the insert
*/
module beltSideSnap(length = 10, lzoom = 1, cantWidth = 4, headL = 6, cantAngle = -10,headHeight = 4, mask = 0, d = 2, res = 30, width = 10)
{
  h = lzoom*hForTaperArm(cantWidth,headL, eps = eps);
  if(mask == 0)
  {
    doubleTaperSnapArm(length, lzoom, cantWidth, headL, cantAngle,headHeight, mask, d,width);
    polydata = [
      [-width/2-h-cantWidth,0],
      [width/2+h+cantWidth,0],
      [width/2,-width],
      [-width/2,-width],
    ];
      translate([0,0,-d/2])
        linear_extrude(height = d)polygon(polydata);
      translate([0,1.5*headL,0])
      {
        cube([d,length+1.5*headL,d], center = true); 
        translate([0,1.5*headL,-d/2])
          color("red") scale([1,.5,1])rotate([0,0,-30])cylinder(d = width, h= d, $fn=3);
      }
  }
  else
  {
  //exth = (mask >0)? d+2*mask : d; 
  //endangle = (mask >0)?90-atan((length+headL)/cantWidth): 1;

    polydata = [
      [-width/2,length+1.75*headL+width],
      [width/2 ,length+1.75*headL+width],
      [width/2+h+cantWidth ,length+1.5*headL],
      [width/2+h+1.5*cantWidth+d ,0],
      [-width/2-h-1.5*cantWidth-d,0],
      [-width/2-h-cantWidth,length+1.5*headL],
    ];
    insidedata = [
      [-width/2+d,length+1.75*headL+width-d],
      [width/2-d ,length+1.75*headL+width-d],
      [width/2+h+cantWidth-d ,length+1.5*headL],
      [width/2+h+1.5*cantWidth ,0],
      [-width/2-h-1.5*cantWidth,0],
      [-width/2-h-cantWidth+d,length+1.5*headL],
    ];
difference()
{
      translate([0,0,d+mask])
        difference()
        {
          translate([0,0.1,-(3*d+2*mask)/2])
            linear_extrude(height = 3*d+2*mask)polygon(polydata);
          //for(y = [0: -length/headL : -length-1.75*headL])
          //translate([0,y,0])
          union()
          {
            doubleTaperSnapArm(length, lzoom, cantWidth, headL, cantAngle,headHeight, mask, d,width);
            translate([0,1.5*headL,0])
            {
              cube([2*(cantWidth+h),length+1.5*headL,d+2*mask], center = true); 
              translate([0,1.75*headL,-(d+2*mask)/2])
                 scale([1,.5,1])rotate([0,0,-30])cylinder(d = width+h+mask,h = d+2*mask, $fn=3);
            }
            //translate([width,length+width/4,-2*d]) color("red")cylinder(d=2*cantWidth, h=10*d, $fn =32);
            //translate([-width,length+width/4,-2*d]) color("red")cylinder(d=2*cantWidth, h=10*d, $fn =32);
          }
        }
union()
{
  headdata = taperArmHeadPoly(l=length, y=cantWidth, lz = lzoom, angle=cantAngle, button  =headHeight, head=headL, mask = 5*mask, eps =eps, smooth = res);
color("blue")
          translate([0,-0.1,(d+mask)/2])
            linear_extrude(height = d+2*mask)polygon(insidedata);
color("green")
          translate([width/2,0,-(d+mask)/2])
            scale([1.1,1,1])
            linear_extrude(height = 4*d+2*mask)polygon(headdata);
          translate([-width/2,0,-(d+mask)/2])mirror([1,0,0])
            scale([1.1,1,1])linear_extrude(height = 4*d+2*mask)polygon(headdata);
}
}
  }
}

/**
 build a tapered snap cantilever arm, with mask >0 build the corresponding mask
*/
module taperSnapArm(length = 10, lzoom = 1, cantWidth = 4, headL = 6, cantAngle = -10,headHeight = 4, mask = 0, d = 2, res = 30)
{
  exth = (mask >0)? d+2*mask : d; 
  endangle = (mask >0)?90-atan((length+headL)/cantWidth): 1;
  h = lzoom*hForTaperArm(cantWidth,length, eps = eps);



//  echo("l = ",length, " y  = ",cantWidth,"  angle = ",cantAngle," tan,",tan(cantAngle)," dy = ",(cantWidth*tan(cantAngle))," l-dy = ",(length+cantWidth*tan(cantAngle))," val = ",(length+cantWidth*tan(cantAngle)-mask));
wd = min(headHeight/4, cantWidth/4);
  data = taperArmPoly(l=length, y=cantWidth, lz = lzoom, angle=cantAngle, button  =headHeight, head=headL, mask = mask, eps =eps, smooth = res);
  //echo("alpha = ",endangle, " res  = ",res,"  data = ",data);
  headdata = taperArmHeadPoly(l=length, y=cantWidth, lz = lzoom, angle=cantAngle, button  =headHeight, head=headL, eps =eps, smooth = res);
  translate([cantWidth,0,-exth/2])
    linear_extrude(height = exth)
    {

     //for(alpha = [0:endangle/res: endangle]) rotate([0,0,alpha])
     //{
        if(mask ==0)
          difference()
          {
            polygon( data );
            polygon( headdata );
          }
        else polygon(data);
      //}
    }
}// module cantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0)

/**
 same  as tapered snap cantilever arm, only mirrored, preparation for belt bucle
  with mask >0 build the corresponding mask
*/
module doubleTaperSnapArm(length = 10, lzoom = 1, cantWidth = 4, headL = 6, cantAngle = -10,headHeight = 4, mask = 0, d = 2, res = 30, width = 10)
{
  disp = (width/2 >cantWidth)? width/2-cantWidth : 1.01*cantWidth;
  translate([disp,0,0])
    taperSnapArm(length, lzoom , cantWidth, headL, cantAngle,headHeight, mask, d, res);
  translate([-disp,0,0])rotate([0,180,0])
    taperSnapArm(length, lzoom , cantWidth, headL, cantAngle,headHeight, mask, d, res);
}


function cantCheckX(x0,x1,tp,minArm = 0.9) = (x1-x0 < minArm*x0*tp)? x1-x0  : minArm*x0*tp; 
function cantileverPoly(x0, x1, y0, y1, tap, mask = 0, minArm = .9) = [ 
  (mask >  0)?[0,0]                :[cantCheckX(x0,x1,tap, minArm),0],
  (mask >  0)?[x0+mask,0]          :[x0,0],
  (mask >  0)?[x0+2*mask,y0-2*mask]:[x0*tap,y0],
  (mask >  0)?[x1+mask,y0-2*mask]  :[x1,y0],
  (mask >  0)?[x1+mask,y0+mask]    :[x1,y0],
  (mask >  0)?[0,y1+y0+2*mask]     :[cantCheckX(x0,x1,tap, minArm),y1+y0]
];


module doubleCantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0, d = 2, minArmRatio = .9)
{
  linear_extrude(height = d)
  {
    polygon(cantileverPoly(baseThick,headThick,cantL,headL,taperratio,mask,minArmRatio));
    mirror([1,0,0]) polygon(cantileverPoly(baseThick,headThick,cantL,headL,taperratio,mask,minArmRatio));
  }

}// module doubleCantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0)

module cantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0, d = 2,minArmRatio = 0.9)
{
  linear_extrude(height = d)
  {
    polygon(cantileverPoly(baseThick,headThick,cantL,headL,taperratio,mask,minArmRatio));
  }
}// module cantilever(baseThick = 2, headThick = 3, cantL = 4, headL = 2, ratio = taperratio, mask = 0)

function roundPoint2D(pt,rad, res = 3,left = false,a0 = 0, a1 = 180) = 
  [
  let (dw = a1-a0) 
  for(n = [((left)? res:0):((left)? -1:1):((left)? 0:res)]) 
let (w = dw/res*n) 
  [pt[0]+rad*cos(a0+w), pt[1]+rad*sin(a0+w)]    
  ]; 
