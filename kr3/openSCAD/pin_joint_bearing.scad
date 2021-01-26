module pin_joint_bearing(width, radius){
    difference(){
        difference(){
            union(){
                translate(v = [-radius/2, 0, 0]){
                    cube([radius, 2*radius, width], center = true);
                }
                cylinder(h = width, r = radius, center = true);
            }
            cube([2.2*radius, 2.2*radius, 0.65*width], center = true);
        }
        translate(v = [0, 0, 0]){
            cylinder(h = 1.1*width, r = 0.4*radius, center = true);
        }
    }
}

/*
UNCOMMENT TO TEST MODULE
pin_joint_bearing(100, 100);
*/