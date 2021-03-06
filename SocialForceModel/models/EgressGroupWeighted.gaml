
model pedestrian_simple_environment

global {
	
	int randomFolder <- rnd(1,100000000);
	
	
	int nb_evacuated_per_step <- 0;
	
	float exit_size <- 108.3 parameter: true;
	float square_area <- 5200 parameter: true;
	float LOS <- 0.5;
	
	int nb_people <- int(square_area/LOS) parameter: true;
	int nb_remaining <- nb_people update: people count(true);
	int nb_remaining_previous <- nb_remaining;
	
	float density <- nb_remaining / square_area;
	
	file buildings_file <- file("../includes/pombal/buildings2.shp") parameter: true;
	file targets_file <- file("../includes/pombal/targets2.shp") parameter: true;
	file praca_file <- file("../includes/pombal/start_area.shp") parameter: true; 
	
	float P_A_pedestrian_SFM parameter: true <- 2.1 category: "SFM" ;
	float P_B_pedestrian_SFM parameter: true <- 0.3 category: "SFM" ;
	float P_ped_influence_radius parameter: true <- 1.0 category: "SFM" ;
	
	float P_A_obstacles_SFM parameter: true <- 10.0 category: "SFM" ;
	float P_B_obstacles_SFM parameter: true <- 0.2 category: "SFM" ;
	float P_obs_influence_radius parameter: true <- 5.0 category: "SFM" ;
	
	float P_Tr_SFM parameter: true <- 0.5 category: "SFM" ;
	float P_shoulder_length <- 0.45 parameter: true;
	
	geometry shape <- envelope(buildings_file);
	geometry free_space <- copy(shape);
	geometry unusable_area;
	
	geometry start_area <- first(praca_file.contents);
	
	float step <- 0.1;
	
	float exit_sum <- 0.0;
	
	//Impactos
	
	// SFM Variables
	float A_ped <- P_A_pedestrian_SFM;
	float D_ped <- P_B_pedestrian_SFM;
	float A_obs <- P_A_obstacles_SFM;
	float D_obs <- P_B_obstacles_SFM;
	float ped_influence_radius <- P_ped_influence_radius;
	float obs_influence_radius <- P_obs_influence_radius;
	float Tr <- P_Tr_SFM;
	float shoulder_length <- P_shoulder_length;
	
	// Group size
	int group_size <- 7  parameter: true;
	
	// Group SFM Params
	float B_group_rep_vis <- 2.0;
	
	float B_group_attr <- 10.0;
	float group_attr_threshold <- (group_size-1)/2;
	float B_group_rep <- 1.0;
	float group_rep_threshold <- shoulder_length + 0.20;
	
	// Outcome variables
	int nb_collisions <- 0;
	float avg_speed <- 0.0;
	
	init {
		
		write "Experiment:" + name;
	
		create obstacle from: buildings_file{
			free_space <- free_space - (shape + P_shoulder_length);
			start_area <- start_area - (shape + P_shoulder_length);
		}
		
		create target from: targets_file{
			exit_ext1 <- shape.points closest_to start_area;
			
			exit_ext2 <- (shape.points - exit_ext1) closest_to start_area;
			if (exit_ext2 = exit_ext1){
				exit_ext2 <- ((shape.points - exit_ext2)-exit_ext1) closest_to start_area;
			}
			
			exit_width <- exit_ext1 distance_to exit_ext2;
			exit_sum <- exit_sum + exit_width;
			write("Exit: " + exit_width);
		}
		
		write("Total exit width: " + exit_sum);
		
		unusable_area <- shape - free_space;
		int nb_groups <- int(nb_people/group_size);
		
		// Create Groups
		loop i from: 0 to: nb_groups{ 
			rgb group_color <- rnd_color(0,240);
			//int group_size <- int(gauss(3, 2));
			
			/*if (group_size < 2){
				group_size <- 2;
			}*/
			
			point group_center <- any_location_in(start_area);
			geometry group_spawn_area <- circle(2, group_center) - unusable_area ;
			target closest_target <- target closest_to group_center;
			
			target group_target <- choose_random_exit();
			
			list<people> group_members <- [];
			
			create people number: group_size {
				group_id <- i;
				color <- group_color;
				location <- any_location_in(group_spawn_area);
				exit_target <- group_target;
				
				group_members << self;
			}
			
			create group{
				id <- i;
				color <- group_color;
				members <- group_members;
				loop member over: members{
					member.group <- self;
				}
			}
			
		}
		
		write("Number of pedestrians: " + length(people));
		//
	}

	float heading(point from, point to){
    	float dY <- to.y-from.y;
    	float dX <- to.x-from.x;
    	return atan2(dY, dX);
    }
    
    target choose_random_exit{
		float random <- rnd(exit_sum);
		float iter_sum <- 0.0;
		loop t over:target{
			iter_sum <- iter_sum + t.exit_width;
			if(random <= iter_sum){
				return t;
			}
		}
	}
	
	reflex stop when: (length(people)/nb_people) <= 0.1  {
		/*ask people{
			do save_positions_to_file;
		}*/
		do pause;
		do die;
	}  
	
	reflex save_data when: every(1#s){
	 	if (cycle = 0){
	 		save ("step,remaining_people,evacuated_people_per_step")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "results.csv" type: "text" rewrite: (cycle = 0) ? true : false;
	 		save ("step,collisions")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "collisions.csv" type: "text" rewrite: (cycle = 0) ? true : false;
	 		save ("")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "trails.txt" type: "text" rewrite: (cycle = 0) ? true : false;
	 		save ("step,avg_speed")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "speed.csv" type: "text" rewrite: (cycle = 0) ? true : false;
	 	}
        save (""+int(cycle*step)+","+length(people)+","+nb_evacuated_per_step) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "results.csv" type: "text" rewrite: false;
        save (""+int(cycle*step)+","+ nb_collisions) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "collisions.csv" type: "text" rewrite: false;
        save (""+int(cycle*step)+","+ avg_speed) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "speed.csv" type: "text" rewrite: false;
        
        nb_evacuated_per_step <- 0;
        //density <- nb_remaining / square_area;
    }
    
     reflex update_groups{
    	ask group{
 			do update;
		}
    }

    reflex calc_social_forces{
    	ask people{//parallel:100{
 			float repx <- 0.0;
 			float repy <- 0.0;

 			point to <- exit_target.centroid;
 			 
 			hd <- 90 - self.heading(to);
 			
     	 	if not (speedX * speedY = 0){
     	 		h <- atan2(speedX, speedY);
     	 	}else{
     	 		h <- hd;
     	 	}
     	 	
     	 	float h1 <- h;
     	 	
     		// Calc attraction to group center (F_att)
     	 	point group_center <- self.group.centroid;
     	 	//centroid(polygon((people where (each.group_id = self.group_id )) collect (each.location)));
     	 	//float angle_to_center
     	 	//float gaze_angle <- h1
     	 	
     	 	if(self distance_to group_center > group_attr_threshold){    	 	
     	 		repx <- repx + B_group_attr * sin( 90 - self.heading(group_center) ) * (1 - cos(90 - self.heading(group_center) - h1));
	       		repy <- repy + B_group_attr * cos( 90 - self.heading(group_center) ) * (1 - cos(90 - self.heading(group_center) - h1));
     	 	}
     	 	//
     	 	
  			// Group alt
  			collisions <- count(people, (each distance_to self < 0.5) and (each.group_id != self.group_id));
	      	ask people at_distance(ped_influence_radius){
	      		if self != myself{
	      		// If group members, rep from them (F_rep)
	       			if self.group_id = myself.group_id {
	       				float distance <- self distance_to myself;
	       				if(distance < group_rep_threshold){
	       					repx <- repx + B_group_rep * sin( 90 - self.heading(myself.location) ) * (1 - cos(90 - self.heading(myself.location) - h1));
	       					repy <- repy + B_group_rep * cos( 90 - self.heading(myself.location) ) * (1 - cos(90 - self.heading(myself.location) - h1));  
	       				}
	      		//
	      			}else{
	       				//Fij
	       				//collisions <- collisions + 1;
	       				repx <- repx + A_ped * exp((shoulder_length - self distance_to myself) / D_ped) * sin( 90 - self.heading(myself.location) ) * (1 - cos(90 - self.heading(myself.location) - h1));
	     	    		repy <- repy + A_ped * exp((shoulder_length - self distance_to myself) / D_ped) * cos( 90 - self.heading(myself.location) ) * (1 - cos(90 - self.heading(myself.location) - h1));
	       			}
	      		}
	      	}
	      	
	      	// Obstacle repulsion
	      	ask obstacle at_distance(ped_influence_radius){

	     		point obs <- (myself closest_points_with self)[1];
	     		
	       		repx <- repx + A_obs * exp((shoulder_length - obs distance_to myself) / D_obs) * sin( 90 -self.heading(obs, myself.location) ) * (1 - cos(90 - self.heading(obs, myself.location) - h1));
	     	    repy <- repy + A_obs * exp((shoulder_length - obs distance_to myself) / D_obs) * cos( 90 -self.heading(obs, myself.location) ) * (1 - cos(90 - self.heading(obs, myself.location) - h1));
	      	}
	      	
			speedX <- speedX + step * (repx + (V0 * sin(hd) - speedX) / Tr);
			speedY <-  speedY + step * (repy + (V0 * cos(hd) - speedY) / Tr);
 			 
    	}
    }
    
    reflex calc_total_collisions when: cycle != 0 and every(1#s){
    	int collisions_sum <- 0;
    	ask people{
    		collisions_sum <- collisions_sum + self.collisions;
    	}
    	//write("Total collisions:" + collisions_sum/2);
    	nb_collisions <- int(collisions_sum/2);
    	//write("Average number of collisions:" + collisions_sum/length(people) );
    }
    
    reflex calc_avg_speed when: cycle != 0 and every(1#s){
    	float abs_speed_sum <- 0.0;
    	ask people{
    		abs_speed_sum <- abs_speed_sum + self.abs_speed();
    	}
    	//write("Total collisions:" + collisions_sum/2);
    	avg_speed <- abs_speed_sum/length(people);
    	//write("Average number of collisions:" + collisions_sum/length(people) );
    }
    
}

species group{
	int id;
	int size;
	rgb color;
	
	list<people> members; 
	
	point centroid;
	
	list<point> trail <- [];
	list<point> positions <- [];
	
	action update{
		members <- members where (not dead(each));
		if length(members) = 0{
			do die;
		}
    		
    	float sum_x <- 0.0;
    	float sum_y <- 0.0;
    	
    	loop member over: people where (each.group_id=id){
    		sum_x <- sum_x + member.location.x;
    		sum_y <- sum_y + member.location.y;
    	}
    	centroid <- point(sum_x/length(members), sum_y/length(members));

    		
    	positions << centroid;
    	if(length(trail)>=10){
			trail >> first(trail);
		}
		trail << centroid;
	}
	
	aspect default {
		if(id != 1){
			return;
		}
		//DRAW TRAIL
		loop i from: 0 to:length(positions)-2{
			if((i+1) < length(positions)){
				geometry trail_line <- line([positions[i],positions[i+1]]);
				draw trail_line color: #black width:10;
			}
		}
		draw line([positions[length(positions)-1],centroid]) color: #black width:10;
	}
	
}

species people{
	int group_id;
	int group_size;
	
	group group;
	
	rgb color;
	//bool exited <- false;
	int collisions <- 0;
	
	float V0 <- 1.34;//gauss(1.34, 0.26);
	
	float speedX;
	float speedY;
	
	float hd;
	float h;
	
	target exit_target;
	list<point> trail <- [];
	list<point> positions <- [];
	
	float heading(point to){
    	float dY <- to.y-self.location.y;
    	float dX <- to.x-self.location.x;
    	return atan2(dY, dX);
    }
    
    float abs_speed{
    	return sqrt(speedX*speedX + speedY*speedY);
    }
    
    reflex save_position when: every(1#s){
    	positions << location;
    	if(length(trail)>=10){
			trail >> first(trail);
		}
		trail << location;
    }
    
	reflex move{
		point next_location <- point(location.x + speedX * step, location.y + speedY * step);
		
		if  not (next_location overlaps free_space){
			speedX <- 0.0;
			speedY <- 0.0;
			return;
		}
	
		location <- next_location;	
	
		if (self distance_to exit_target <= 0) {
			nb_evacuated_per_step <- nb_evacuated_per_step + 1;
			//do save_positions_to_file;
			do die;
		}
	}
	
	action save_positions_to_file{
		string positions_line <- ""+group_id+" ";
		loop pos over: positions{
			positions_line <- positions_line + pos + " ";
		}
		save (positions_line) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "trails.txt" type: "text" rewrite: false;
	}
	
	aspect default {
		draw triangle(shoulder_length) color: color border: color rotate: -h - 180;
		
		//DRAW TRAIL
		if(group_id != 1){
			return;
		}
		loop i from: 0 to:length(positions)-2{
			if((i+1) < length(positions)){
				geometry trail_line <- line([positions[i],positions[i+1]]);
				draw trail_line color: color;
			}
		}
		draw line([positions[length(positions)-1],location]) color: color;
		
	}
}

species obstacle {
	
	float heading(point from, point to){
    	float dY <- to.y-from.y;
    	float dX <- to.x-from.x;
    	return atan2(dY, dX);
    }
	
	aspect default {
		draw shape color: #gray border: #black;
	}
}

species target {
	
	point exit_ext1;
	point exit_ext2;
	
	float exit_width;
	
	point centroid <- centroid(shape);
	
	aspect default {
		draw shape color: #red border: #black;
	}
}

experiment main1 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 1;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main2 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 2;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main3 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 3;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main4 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 4;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main5 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 5;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main6 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 6;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

experiment main7 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size <- 7;
	
	output {
		display map{
			species obstacle;
			species group;
			species people;
			species target;
		}
	}
}

