difference(){
   wheel_r = 7;
   thickness = 3;
   
   union(){
      //Front face of wheel   
      cylinder(r=wheel_r, h=1, $fn=90);
      //post in the middle
      cylinder(r=3.4, h=3, $fn=90);
      difference(){
         cylinder(r=wheel_r, h=6 ,$fn=90);
         cylinder(r=wheel_r-1.5, h=6.5 ,$fn=90);
      }
   }
   
   //pin in the middle
   cylinder(r=1.2, h=thickness, $fn=30);

   //crosspieces
   hole_depth = 2;
   translate([0,0,thickness-(hole_depth/2)]){
     cube([2, 4, hole_depth], center=true);
     cube([4, 2, hole_depth], center=true);
   }
}
    