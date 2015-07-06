use <Write.scad>

$fn=500;


x=100;
grosor=2;

ladox = 110;
ladoy = 110;
alturaz = 95;
alturaz2 = 12;
radio = 2;



module borde(){
    hull(){  
		translate([-3,0,0 ]) 
        cube([6,ladoy, alturaz2], center=true );
        
     	translate([5/2,0,0])        
        cube([1,ladoy, alturaz2], center=true );
    }
}

module caja(){

    difference(){
        cube([ladox,ladoy,alturaz], center=true);

            //%translate([0,0,-grosor])
//                color("red") 
//                    cube([ladox-grosor,ladoy-grosor,alturaz],                   center=true);
    }
    
    translate([ladox/2 - 3,0,-alturaz/2 + 6])
        borde();
    translate([-ladox/2 + 3,0,-alturaz/2 + 6])
        rotate([0,0,180])
            borde();
    translate([0,ladoy/2-3,-alturaz/2 + 6])
        rotate([0,0,90])
            borde();
    translate([0,-ladoy/2+3,-alturaz/2 + 6])
        rotate([0,0,-90])
            borde();
    
    
}

module caja_modificada(){
    difference(){
        caja();
        
        translate([ladox/2 -10/2 -20,55,-alturaz/2 +7 +15/2])
            color("green")
                cube([15,25,15],center=true);
        
        translate([ladox/2 -14/2 -52,+65,-alturaz/2 + 10/2 +9])
            rotate ([90,0,0])
                color("red")
                    cylinder(r=10/2, h=25,$fn=500);
        
        translate([-ladox/2 +10/2 +18,-58,-alturaz/2 + 8/2 +12.5])
            rotate ([90,0,0])
                color("blue")
                    cylinder(r=8/2, h=15, center= true);
        
        translate([-ladox/2 +10/2 +18,-59,-alturaz/2 - 18/2 +12.5])
            rotate ([90,0,0])
                color("blue")
                    cube([8,25,15], center = true);
        
        translate([ladox/2 -5, ladoy/2 -5,-alturaz/2 +6 ])
            color("green")
                cylinder(r=radio, h=13, center= true);
        translate([ladox/2 -5, -ladoy/2 +5,-alturaz/2 +6 ])
            color("green")
                cylinder(r=radio, h=13, center= true);
        translate([-ladox/2 +5, ladoy/2 -5,-alturaz/2 +6 ])
            color("green")
                cylinder(r=radio, h=13, center= true);
        translate([-ladox/2 +5, -ladoy/2 +5,-alturaz/2 +6 ])
            color("green")
                cylinder(r=radio, h=13, center= true);
        
        
     
    }
}


//caja_modificada();

module puzzle_caja_lado1(){

	difference(){
		caja_modificada();
		translate([9,0,0])
			cube([ladox+1,ladoy+1,alturaz+1], center=true);
		translate([grosor,0,alturaz2 ])
			cube([ladox+1.5,ladoy+1.5,alturaz+1], center=true);
		for( i = [-alturaz/2 +20 :10:  alturaz/2])
			{
				translate([-ladox/2 -grosor, ladoy/2 - grosor ,i])
				cube([grosor+2, grosor+2, 5]);
				
			}
		for( i = [-alturaz/2 +20 :10:  alturaz/2])
			{
				translate([-ladox/2 -grosor, -ladoy/2 - grosor ,i])
				cube([grosor+2, grosor+2, 5]);
				
			}
		for( i = [-ladoy/2 -1.5:10:  ladoy/2])
			{
				translate([-ladox/2 -grosor, i , alturaz/2 -grosor])
				cube([grosor+2, 5, grosor +2]);
				
			}
	}


}

module puzzle_caja_lado2(){
difference(){
		caja_modificada();
		translate([-9,0,0])
			cube([ladox+1,ladoy+1,alturaz+1], center=true);
		translate([-grosor,0,alturaz2])
			cube([ladox+1.5,ladoy+1.5,alturaz+1], center=true);
		for( i = [-alturaz/2 +20 :10:  alturaz/2])
			{
				translate([ladox/2 -grosor, ladoy/2 - grosor ,i])
				cube([grosor+2, grosor+2, 5]);
				
			}
		for( i = [-alturaz/2 +20 :10:  alturaz/2])
			{
				translate([+ladox/2 -grosor, -ladoy/2 - grosor ,i])
				cube([grosor+2, grosor+2, 5]);
				
			}
		for( i = [-ladoy/2 -1.5:10:  ladoy/2])
			{
				translate([ladox/2 -grosor, i , alturaz/2 -grosor])
				cube([grosor+2, 5, grosor +2]);
				
			}
	}

}

module puzzle_caja_lado3(){
	difference(){
		caja_modificada();
		puzzle_caja_lado1();
		puzzle_caja_lado2();
		translate([0,9,0])
			cube([ladox+1,ladoy+1,alturaz+1], center=true);
		translate([0,grosor,alturaz2])
			cube([ladox+1.5,ladoy+1.5,alturaz+1], center=true);
		for( i = [-ladox/2 -1.5 :10:  ladox/2])
			{
				translate([i, -ladoy/2 -grosor, alturaz/2 -grosor])
				cube([5, grosor+2, grosor +2]);
				
			}

	}
}

module puzzle_caja_lado4(){
	difference(){
		caja_modificada();
		puzzle_caja_lado1();
		puzzle_caja_lado2();
		translate([0,-9,0])
			cube([ladox+1,ladoy+1,alturaz+1], center=true);
		translate([0,-grosor,alturaz2])
			cube([ladox+1.5,ladoy+1.5,alturaz+1], center=true);
		for( i = [-ladox/2 -1.5 :10:  ladox/2])
			{
				translate([i, ladoy/2 -grosor, alturaz/2 -grosor])
				cube([5, grosor+2, grosor +2]);
				
			}

	}
}

module tapa(){
	difference(){
		caja_modificada();
		puzzle_caja_lado1();
		puzzle_caja_lado2();
		puzzle_caja_lado3();
		puzzle_caja_lado4();
	}

}
rotate([0,-90,0])
puzzle_caja_lado1();
//caja_modificada();


