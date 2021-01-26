module pin_joint_knuckle(width, radius){
    diameter = 2*radius;
    union(){
        cylinder(1.1*width, 0.38*radius, 0.38*radius, center = true);
        cylinder(0.6*width, radius, radius, center = true);
        translate(v = [radius/2, 0, 0]){
            cube([radius, diameter, 0.6*width], center = true);
        }
    }
}

/*
UNCOMMENT TO TEST MODULE
pin_joint_knuckle(100, 100);
*/