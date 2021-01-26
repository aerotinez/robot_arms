module swivel_joint_bearing (height, radius, chamfer_size){
    difference(){
        difference(){
            cylinder(h = height, r = radius, center = true);
            cylinder(h = 1.1*height, r = 0.5*radius, center = true);
        }
        
        cs = chamfer_size;
        
        translate(v = [0, 0, height/2]){
            rotate_extrude(convexity = 10){
                translate(v = [radius, 0, 0]){
                    rotate([0, 0, 45]){
                        square(size = [cs, cs], center = true);
                    }
                }
            }
        }
    }
}

/*
// UNCOMMENT TO TEST THIS MODULE
swivel_joint_bearing(100, 100, 10);
*/