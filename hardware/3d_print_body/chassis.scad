
difference(){
    base_h = 20;
    base_r = 27;
    union(){
        //base shape
        translate([base_r, base_r, 0]){
            cylinder(r=base_r, h=base_h);
        }
        
        //motor mount block
        translate([base_r-9,0,0]){
            cube([18, base_r*2, 8]);
        }
        
        //LED ring support
        translate([base_r/2-2.5, base_r/2-2.5 ,base_h]){
            led_struts();
        }
    }
    
    //battery cutout
    w = base_r * 2;
    l = base_r * 2;
    translate([(w-42)/2, (l-25)/2, base_h-9]){
        cube([42, 25, 9]);
    }
    translate([w-7, w/2 + 25/2-4, base_h-9]){
        //wire cutout
        cube([8,4,9]);
    }
   
   //motors
    union(){
        translate([base_r, 0,4]){
            translate([0,w,0]){
                rotate([90, 0,0])
                {
                    motor();
                }
            }
            translate([0,0,0]){
                rotate([-90,0,0])
                {
                    motor();
                }
            }
        }
    } 
    
    //Wire holes
    translate([base_r-10,base_r+18,0]){
        cylinder(h=base_h, r=2);
    }
    translate([base_r-10,base_r-18,0]){
        cylinder(h=base_h, r=2);
    }
    
    //Wire channels
    translate([base_r-2, base_r-14,0])
    {
        cube([4,10,8]);
    }
    translate([base_r-2, base_r+4,0])
    {
        cube([4,10,8]);
    }
    translate([base_r-12, base_r-18,0])
    {
        cube([4,14,8]);
    }
    translate([base_r-12, base_r+4,0])
    {
        cube([4,14,8]);
    }
    translate([base_r-12, base_r-8,0])
    {
        cube([14,4,8]);
    }
    translate([base_r-12, base_r+4,0])
    {
        cube([14,4,8]);
    }
    
}

module motor(){
    intersection()
    {
        translate([0,0,8]){
            cube([10, 8, 16], center=true);
        }
        //todo 16 here is a guess
        cylinder(r=5, h=16, $fn=40); 
    }
}

module strut(){
    height=20;
    union(){
    cylinder(h=height, r=2.5, $fn=10);
        translate([0,0,height]){
            cylinder(h=4, r=1, $fn=10);
        }
    }
}

module led_struts(){
    translate([0,0,0]){
        strut();
    }
    translate([32,0,0]){
        strut();
    }
    translate([32,32,0]){
        strut();
    }
    translate([0,32,0]){
        strut();
    }
}


