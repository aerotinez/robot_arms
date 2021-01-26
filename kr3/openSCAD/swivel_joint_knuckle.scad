module swivel_joint_knuckle(height, radius, chamfer_size){
    difference(){
        union(){
            translate(v = [0, 0, height/2]){
                cylinder(h = height, r = radius, center = true);
            }
            translate(v = [0, 0, -height/2]){
                cylinder(h = height, r = 0.45*radius, center = true);
            }
        }
        
        cs = chamfer_size;
        
        rotate_extrude(convexity = 10){
            translate(v = [radius, 0 ,0]){
                rotate([0, 0, 45]){
                    square(size = [cs, cs], center = true);
                }
            }
        }
    }
}

/*
//  UNCOMMENT TO TEST MODULE
swivel_joint_knuckle(50, 100, 5);
*/