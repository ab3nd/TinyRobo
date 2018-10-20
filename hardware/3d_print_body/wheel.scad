difference(){
    thickness=3;
    //Main outer wheel
    cylinder(r=6, h=thickness,$fn=90);
    
    //pin in the middle
    cylinder(r=0.7, h=thickness, $fn=30);
    
    //crosspieces
    //todo 2 is a guess
    hole_depth = 2;
    translate([0,0,thickness-(hole_depth/2)]){
        cube([1.4, 3.85, hole_depth], center=true);
        cube([3.85, 1.4, hole_depth], center=true);
    }
}
    