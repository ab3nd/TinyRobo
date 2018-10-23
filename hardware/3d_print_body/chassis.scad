
difference(){
    base_h = 22;
    base_r = 27;
    union(){
        //base shape
        translate([base_r, base_r, 0]){
            cylinder(r=base_r, h=base_h, $fn=90);
        }
        
        //motor mount block
        translate([base_r-9,0,0]){
            cube([18, base_r*2, 8]);
        }
        
        //LED ring support
        translate([base_r/2.7, base_r/2-2.7 ,base_h]){
            led_struts();
        }
    }
    
    //battery cutout
    w = base_r * 2;
    l = base_r * 2;
    translate([(w-43)/2, (l-26)/2, base_h-9]){
        cube([43, 26, 9]);
    }
    translate([w-7, w/2 + 26/2-4, base_h-9]){
        //wire cutout
        cube([8,4,9]);
    }
   
   //motors
    union(){
        translate([base_r, 0,5]){
            translate([0,w+0.02,0.5]){
                rotate([90, 90,0])
                {
                    motor();
                }
            }
            translate([0,-0.02,0.5]){
                rotate([-90,90,0])
                {
                    motor();
                }
            }
        }
    } 
    
    //Wire holes
    translate([base_r-10,base_r+18,0]){
        cylinder(h=base_h, r=2, $fn=40);
    }
    translate([base_r-10,base_r-18,0]){
        cylinder(h=base_h, r=2, $fn=40);
    }
    
    //Wire channels
    translate([base_r-2, base_r-14,0])
    {
        cube([4,10,11]);
    }
    translate([base_r-2, base_r+4,0])
    {
        cube([4,10,11]);
    }
    translate([base_r-12, base_r-18,0])
    {
        cube([4,14,11]);
    }
    translate([base_r-12, base_r+4,0])
    {
        cube([4,14,11]);
    }
    translate([base_r-12, base_r-8,0])
    {
        cube([14,4,11]);
    }
    translate([base_r-12, base_r+4,0])
    {
        cube([14,4,11]);
    }
    
}

module motor(){
    intersection()
    {
        translate([0,0,9]){
            cube([11, 8.1, 18], center=true);
        }
        cylinder(r=5.55, h=18, $fn=40); 
    }
}

module strut(){
    height=20;
    union(){
    cylinder(h=height, r=2.5, $fn=40);
        translate([0,0,height]){
            cylinder(h=4, r=0.8, $fn=40);
        }
    }
}

module led_struts(){
    translate([0,0,0]){
        strut();
    }
    translate([32.5,0,0]){
        strut();
    }
    translate([32.5,32.5,0]){
        strut();
    }
    translate([0,32.5,0]){
        strut();
    }
}


