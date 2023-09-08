
model pedestrian_simple_environment

global {
	
	int randomFolder <- rnd(1,100000000);
	
	
	int nb_evacuated_per_step <- 0;
	
	float exit_size <- 108.3 parameter: true;
	float square_area <- 5200 parameter: true;
	float LOS <- 4;
	
	//int nb_people <- int(square_area/LOS) parameter: true;
	int nb_people <- (18250 - 230) parameter: true;
	int nb_remaining <- nb_people update: people count(true);
	int nb_remaining_previous <- nb_remaining;
	
	float density <- nb_remaining / square_area;
	
	/* File params */
	file buildings_file <- file("../includes/comercio/buildings2.shp") parameter: true;
	file targets_file <- file("../includes/pombal/targets2.shp") parameter: true;
	file praca_file <- file("../includes/pombal/start_area.shp") parameter: true; 
	
	/* SFM params */
	float P_A_pedestrian_SFM parameter: true <- 2.1 category: "SFM" ;
	float P_B_pedestrian_SFM parameter: true <- 0.3 category: "SFM" ;
	float P_ped_influence_radius parameter: true <- 1.0 category: "SFM" ;
	
	float P_A_obstacles_SFM parameter: true <- 10.0 category: "SFM" ;
	float P_B_obstacles_SFM parameter: true <- 0.2 category: "SFM" ;
	float P_obs_influence_radius parameter: true <- 2.0 category: "SFM" ;
	
	float P_Tr_SFM parameter: true <- 0.5 category: "SFM" ;
	float P_shoulder_length <- 0.45 parameter: true;
	
	/*  Group SFM Params */
	//float line_of_sight_angle <- 90;
	//float B_group_vis <- 4;
	
	float B_group_attr <- 10.0;
	float group_attr_threshold <- (group_size_avg-1)/2;
	
	float B_group_rep <- 1.0;
	float group_rep_threshold <- shoulder_length + 0.20;
	
	int group_size_avg <- 3  parameter: true;
	
	list<float> group_size_dist <- [0, 0.613899614, 0.214859647, 0.113534384, 0.057706355] parameter: true; //from 2 to 5
	list<float> group_speed_values <- [0, 1.08, 0.99, 0.9775, 0.935] parameter: true; 
	
	/*  Behavior params */
	list<float> exit_attr <- [25.0, 25.0, 25.0, 25.0] parameter: true;
	float egress_interval_period <- 0.01 #hours parameter: true; //3.5 hours
	
	//float avg_group_speed <- 1.34;
	//float std_dev_group_speed <- 0.37;
	
	//float avg_group_speed <- 1.34;
	//float std_dev_group_speed <- 0.26;
	
	geometry shape <- envelope(buildings_file);
	geometry free_space <- copy(shape);
	geometry unusable_area;
	geometry start_area <- first(praca_file.contents);
	float step <- 0.1;
	float exit_sum <- 0.0;
	
	//Impactos
	
	/* SFM Variables */
	float A_ped <- P_A_pedestrian_SFM;
	float D_ped <- P_B_pedestrian_SFM;
	float A_obs <- P_A_obstacles_SFM;
	float D_obs <- P_B_obstacles_SFM;
	float ped_influence_radius <- P_ped_influence_radius;
	float obs_influence_radius <- P_obs_influence_radius;
	float Tr <- P_Tr_SFM;
	float shoulder_length <- P_shoulder_length;
	
	
	// Outcome variables
	int nb_collisions <- 0;
	float avg_speed <- 0.0;
	
	//Performance measure (time to complete first step)
	float start_time;
	float end_time;
	float cycle_time;
	float sim_time;
	
	//group id in the center of the map
	int center_group_id <- 0;
	int center_group_id_2 <- 0;
	int center_group_id_3 <- 0;
	
	rgb background_color <- #white;
	float cycle_duration <- 0.000001#seconds;
	

	float avg_social_time <- 0.0;
	
	init {
		
		write "Experiment:" + name;
		write egress_interval_period;
		
		write("Group Attraction Threshold: " + group_attr_threshold);
	
		create obstacle from: buildings_file{
			free_space <- free_space - (shape + P_shoulder_length/2);
			start_area <- start_area - (shape + P_shoulder_length);
		}
		
		int nb_targets <- length(targets_file);
		float default_weight <- 100/nb_targets;
		
		create target from: targets_file{
			exit_ext1 <- shape.points closest_to start_area;
			
			exit_ext2 <- (shape.points - exit_ext1) closest_to start_area;
			if (exit_ext2 = exit_ext1){
				exit_ext2 <- ((shape.points - exit_ext2)-exit_ext1) closest_to start_area;
			}
			
			exit_width <- exit_ext1 distance_to exit_ext2;
			
			float nb_entry_points <- exit_width/4;
			float offsetX <- (exit_ext2.x - exit_ext1.x) / nb_entry_points;
			float offsetY <- (exit_ext2.y - exit_ext1.y) / nb_entry_points;
			
			loop i from: 1 to: nb_entry_points-1{
				float x <- exit_ext1.x;
				float y <- exit_ext1.y;
				exit_points << point([x+i*offsetX,y+i*offsetY]);
			}
			
			
			exit_sum <- exit_sum + exit_width;
			
			if id > 0 and nb_targets < length(exit_attr) {
				write(id);
				weight <- exit_attr[id-1];
			}else{
				weight <- default_weight;
			}
			
			write("Exit " + id + ": " + exit_width);
		}
		
		write("Total exit width: " + exit_sum);
		
		unusable_area <- shape - free_space;
		
		int size2 <- 0;
		int size3 <- 0;
		int size4 <- 0;
		int size5 <- 0;
		
		int i <- 0;
		loop while: length(people) < nb_people {
			rgb group_color <- rnd_color(150,240); //160
			int group_size <- weighted_random_group_size();
			
			/*point start_center <- start_area.centroid;
			float mean_x <- start_center.x;
			float mean_y <- start_center.y;
			float std_dev <- 2;
			point group_center <- point(gauss(mean_x, std_dev), gauss(mean_y, std_dev));*/
			
			point group_center <- any_location_in(start_area);
			geometry group_spawn_area <- circle(1, group_center) - unusable_area ;
			
			target group_target <- weighted_random_exit();
			
			list<people> group_members <- [];
			
			float group_speed <- group_speed_values[group_size - 1];
			
			create people number: group_size {
				V0 <- group_speed;
				group_id <- i;
				color <- group_color;
				location <- any_location_in(group_spawn_area);
				exit_target <- group_target;
				target_point <- exit_target.centroid;//exit_target.exit_points[rnd(0, length(exit_target.exit_points)-1)];
				group_members << self;
			}
			
			create group{
				id <- i;
				color <- group_color;
				size <- group_size;
				speed <- group_speed;
				group_attr_threshold <- (size-1)/2;
				members <- group_members;
				
				loop member over: members{
					member.group <- self;
				}
				
				do update;
				
				int rand_time <- int(rnd(0, egress_interval_period));
				social_time <- ((rand_time^2) / (egress_interval_period^2)) * egress_interval_period;
				avg_social_time <- avg_social_time + social_time;
			}
			
			if(group_size = 2){
				size2 <- size2 + 1;
			}else if(group_size = 3){
				size3 <- size3 + 1;
			}else if(group_size = 4){
				size4 <- size4 + 1;
			}else if(group_size = 5){
				size5 <- size5 + 1;
			}
			
			i <- i + 1;
		}
		
		write("Size 2: " + (size2/i)*100 +"%");
		write("Size 3: " + (size3/i)*100 +"%");
		write("Size 4: " + (size4/i)*100 +"%");
		write("Size 5: " + (size5/i)*100 +"%");
		
		
		write("Avg Social Time: "+ (avg_social_time / length(group)));
		
		point map_center <- centroid(start_area);
		
		list<group> center_groups <- (group closest_to (map_center, 3)); 
		
		center_group_id <- center_groups[0].id; 
		center_group_id_2 <- center_groups[1].id; 
		center_group_id_3 <- center_groups[2].id; 
		
		/* 
		center_group_id <- (people closest_to map_center).group_id; */
		(group where( each.id = center_group_id ))[0].social_time <- 0.0;
		(group where( each.id = center_group_id_2 ))[0].social_time <- 0.0;
		(group where( each.id = center_group_id_3 ))[0].social_time <- 0.0;
		
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

	int weighted_random_group_size{
		float random <- rnd(100)/100;
		float iter_sum <- 0.0;
		int i <- 0;
		loop prob over:group_size_dist{
			iter_sum <- iter_sum + prob;
			if(random <= iter_sum){
				return i + 1;
			}
			i <- i + 1;
		}
	}
	
	target weighted_random_exit{
		float random <- rnd(100);
		float iter_sum <- 0.0;
		loop t over:target{
			iter_sum <- iter_sum + t.weight;
			if(random <= iter_sum){
				return t;
			}
		}
	}
	
	reflex stop when: (length(people)/nb_people) <= 0.01  {
		/*ask people{
			do save_positions_to_file;
		}*/
		end_time <- machine_time;
		sim_time <- end_time - start_time;
		write(sim_time);
		do pause;
		//do die;
	}  
	
//	reflex save_cycle_time when: cycle < 3{
//		if (cycle = 1){
//			start_time <- machine_time;
//		}
//		else if(cycle = 2){
//			end_time <- machine_time;
//			cycle_time <- end_time - start_time;
//			save (cycle_time)  to: "group/" + experiment.name + "/performance/" +  nb_people+ "/"+ "time.txt" type: "text";
//			write(cycle_time);		
//		}
//	}
	
//	reflex save_data when: every(1#s){
//	 	if (cycle = 0){
//	 		save ("step,remaining_people,evacuated_people_per_step")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "results.csv" type: "text" rewrite: (cycle = 0) ? true : false;
//	 		save ("step,collisions")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "collisions.csv" type: "text" rewrite: (cycle = 0) ? true : false;
//	 		save ("")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "trails.txt" type: "text" rewrite: (cycle = 0) ? true : false;
//	 		save ("step,avg_speed")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "speed.csv" type: "text" rewrite: (cycle = 0) ? true : false;
//	 	}
//        save (""+int(cycle*step)+","+length(people)+","+nb_evacuated_per_step) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "results.csv" type: "text" rewrite: false;
//        save (""+int(cycle*step)+","+ nb_collisions) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "collisions.csv" type: "text" rewrite: false;
//        save (""+int(cycle*step)+","+ avg_speed) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "speed.csv" type: "text" rewrite: false;
//        
//        nb_evacuated_per_step <- 0;
//        //density <- nb_remaining / square_area;
//    }
    
	reflex save_egress_data when: every(5 #minutes){
	 	if (cycle = 0){
	 		save ("step,remaining_people,evacuated_people_per_step")  to: "group/" + experiment.name + "/" + randomFolder + "/"+ "results.csv" type: "text" rewrite: (cycle = 0) ? true : false;
	 	}
        save (""+int(cycle*step)+","+length(people)+","+nb_evacuated_per_step) to: "group/" + experiment.name + "/"+ randomFolder +"/"+ "results.csv" type: "text" rewrite: false;
        
        nb_evacuated_per_step <- 0;
        //density <- nb_remaining / square_area;
    }
    
     reflex update_groups{
    	ask group parallel:50{
 			do update;
		}
    }

    reflex calc_social_forces{
    	ask people parallel:100{
    		
    		float sum_forces_x <- 0.0;
 			float sum_forces_y <- 0.0;
    		
    		point target_point <- self.target_point;
 			float target_hd <- self.heading(target_point);
 			float hd <- target_hd;
    		
    		if (cycle * step) < self.group.social_time{
    			V0 <- 0.0;
    			hd <- self.heading(self.group.centroid);
    		}else{
    			V0 <- self.group.speed;
    		}
    		
     	 	//Check if there are obstacles. 
     	 	//If there is an obstacle nearby, the agent chooses a perpendicular direction until the obstacle is no longer in the path
     	 	//get heading to obstacle
     	 	//sum 90º to heading (new heading)
     	 	
     	 	obstacle close_obstacle <- obstacle closest_to self;
     	 	if (close_obstacle distance_to self) < 2 {
     	 		geometry path_line <- line([self.location, target_point]) + shoulder_length;
     	 		
     	 		if(path_line intersects close_obstacle){
     	 			point obs <- (self closest_points_with close_obstacle)[1];
	       	 		float obs_heading <- close_obstacle.heading(self.location, obs);
	       	 		float obs_group_heading <- close_obstacle.heading(self.group.centroid, obs);
	       	 		
	       	 		float new_heading_1 <- obs_group_heading + 90;
	       	 		float new_heading_2 <- obs_group_heading - 90;
	       	 		
	       	 		if abs(new_heading_1 - hd) < abs(new_heading_2 - hd){
	       	 			hd <- obs_heading + 120;
	       	 		}else{
	       	 			hd <- new_heading_2 - 120;
	       	 		}
	       	 		
	       	 		
	       	 		// if one angle is negative and the other is positive:
	       	 			//
//	       	 		if hd > obs_heading{
//	       	 			hd <- obs_heading + 110;
//	       	 		}else{
//	       	 			hd <- obs_heading - 110;
//	       	 		}
     	 		}
     	 	}
     	 	
     	 	if not (speedX * speedY = 0){
     	 		heading <- atan2(speedY, speedX);
     	 	}else{
     	 		heading <- hd;
     	 	}
     	 	
     	 	if (cycle * step) < self.group.social_time{
     	 		heading <- self.heading(self.group.centroid); 
     	 	}
     	 	
     	 	float h1 <- heading;
     	 	
     	 	point group_center <- self.group.centroid;
     	 	
     	 	//Calculation of the group cohesion forces (attraction forces to the group center of mass)  
     	 	if(self distance_to group_center > self.group.group_attr_threshold){
     	 		    	 	
     	 		sum_forces_x <- sum_forces_x + B_group_attr * cos( self.heading(group_center) );
     	 		
	       		sum_forces_y <- sum_forces_y + B_group_attr * sin( self.heading(group_center) );
     	 	}
     	 	
     	 	//Calculation of the group line of sight adjustment forces 
//     	 	float sight <- abs(self.heading(group_center)) - line_of_sight_angle/2;	
//     	 	if sight < 0 {
//     	 		sight <- 0;
//     	 	} 	
//     	 	sum_forces_x <- sum_forces_x + -B_group_vis * sight * speedX;
//     	 	sum_forces_y <- sum_forces_y + -B_group_vis * sight * speedY;
     	 	
  			// Collisions calculation
  			// A collision is detected when a neighbour is less than 0.5 meters close. 
  			//collisions <- count(people, (each distance_to self < 0.5) and (each.group_id != self.group_id));
  			
  			//Query the pedestrian agents who are at the influence radius of the current agent
	      	ask people at_distance(ped_influence_radius){
	      		if self != myself{
	      			// Calculate the inter-group repulsion forces if a neighbour pedestrian is a group member 
	      			if self.group_id = myself.group_id {
	       				float distance <- self distance_to myself;
	       				// Only apply intra-group repulsion if the group member is close enough
	       				if(distance < group_rep_threshold){
	       					
	       					sum_forces_x <- sum_forces_x + B_group_rep * cos(self.heading(myself.location) ) ;
	       					//* (1 - sin(self.heading(myself.location) - h1));
	       					sum_forces_y <- sum_forces_y + B_group_rep * sin(self.heading(myself.location) ) ;
	       					//* (1 - sin(self.heading(myself.location) - h1));  
	       				}
	      			//Else, calculate the inter-person repulsion forces with non-group members (classic SFM)
	      			}else{

	       				sum_forces_x <- sum_forces_x + A_ped * exp((shoulder_length - self distance_to myself) / D_ped) * 
	       				cos(self.heading(myself.location) ) * (1 - sin(self.heading(myself.location) - h1));
	       				
	     	    		sum_forces_y <- sum_forces_y + A_ped * exp((shoulder_length - self distance_to myself) / D_ped) 
	     	    		* sin(self.heading(myself.location) ) * (1 - sin(self.heading(myself.location) - h1));
	       			}
	      		}
	      	}
	      	
	      	// Calculation of obstacle repulsion forces (classic SFM) within influence range
	      	ask obstacle at_distance(ped_influence_radius){

	     		point obs <- (myself closest_points_with self)[1];
	     		
	       		sum_forces_x <- sum_forces_x + A_obs * exp((shoulder_length - obs distance_to myself) / D_obs) * 
	       		cos( self.heading(obs, myself.location) ) * (1 - sin(self.heading(obs, myself.location) - h1));
	       		
	     	    sum_forces_y <- sum_forces_y + A_obs * exp((shoulder_length - obs distance_to myself) / D_obs) 
	     	    * sin(self.heading(obs, myself.location) ) * (1 - sin(self.heading(obs, myself.location) - h1));
	      	}
	      	
	      	//update of X and Y speed components
			speedX <- speedX + step * (sum_forces_x + (V0 * cos(hd) - speedX) / Tr);
			speedY <-  speedY + step * (sum_forces_y + (V0 * sin(hd) - speedY) / Tr);
 			 
    	}
    }
    
//    reflex calc_total_collisions when: cycle != 0 and every(1#s){
//    	int collisions_sum <- 0;
//    	ask people{
//    		collisions_sum <- collisions_sum + self.collisions;
//    	}
//    	//write("Total collisions:" + collisions_sum/2);
//    	nb_collisions <- int(collisions_sum/2);
//    	//write("Average number of collisions:" + collisions_sum/length(people) );
//    }
    
//    reflex calc_avg_speed when: cycle != 0 and every(1#s){
//    	float abs_speed_sum <- 0.0;
//    	ask people{
//    		abs_speed_sum <- abs_speed_sum + self.abs_speed();
//    	}
//    	//write("Total collisions:" + collisions_sum/2);
//    	avg_speed <- abs_speed_sum/length(people);
//    	//write("Average number of collisions:" + collisions_sum/length(people) );
//    }
    
//    reflex save_data{
//        save (cycle) to: "C:/cycle/cycle.txt" type: "text" rewrite: true;      
//    }
    
//    reflex change_background  when: cycle != 0 and every(1#s){
//    	background_color <- #white;
//    }
    
}

species group{
	int id;
	int size;
	float speed;
	rgb color;
	
	float social_time;
	float group_attr_threshold;
	
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
    	
    	loop member over: members{
    		sum_x <- sum_x + member.location.x;
    		sum_y <- sum_y + member.location.y;
    	}
    	centroid <- point(sum_x/length(members), sum_y/length(members));
    	location <- centroid;

    	positions << centroid;
    	if(length(trail)>=10){
			trail >> first(trail);
		}
		trail << centroid;
	}
	
	aspect default {
		if(id != center_group_id){
			return;
		}
		//DRAW TRAIL
		/*loop i from: 0 to:length(positions)-2{
			if((i+1) < length(positions)){
				geometry trail_line <- line([positions[i],positions[i+1]]);
				draw trail_line color: #black width:10;
			}
		}*/
		
//		loop i from: 0 to:length(positions)-2{
//			draw circle(0.02, positions[i]) color: #purple;
//			
//			if((i+1) < length(positions)){
//				geometry trail_line <- line([positions[i],positions[i+1]]);
//				rgb line_color <- get_speed_color(positions[i] distance_to positions[i+1]); 
//				draw trail_line color: line_color width:100;
//			}
//		}
		//draw circle(0.15, centroid) color: #purple;
		//draw line([positions[length(positions)-1],centroid]) color: #black width:10;
	}
	
}

species people{
	int group_id;
	int group_size;
	
	group group;
	
	rgb color;
	int collisions <- 0;
	
	float V0 <- 1.34;//gauss(1.34, 0.26);
	
	float speedX;
	float speedY;
	
	float heading;
	
	target exit_target;
	point target_point; 	
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
    
    reflex save_position when: every(0.2 #s){
	    if(group_id != center_group_id and group_id != center_group_id_2 and group_id != center_group_id_3){
	    	return;
	    }
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
	
	rgb get_speed_color(float distance){
		float instant_speed <- distance/step;
		float percentage <- instant_speed/V0;
		
		int r <- int(255 * percentage);
		int b <- int(255 * (1-percentage));
		
		return rgb(r,0,b);
	}
	
	aspect default {
		rgb ped_color;
		if(group_id != center_group_id and group_id != center_group_id_2 and group_id != center_group_id_3){
			if(V0 = 0){
				ped_color <- #white;
			}else{
				ped_color <- color;
			}
			
		}else{
			if(group_id = center_group_id){
				ped_color <- #green;
			}else if(group_id = center_group_id_2){
				return;
				ped_color <- #blue;
			}else if(group_id = center_group_id_3){
				return;
				ped_color <- #red;
			}
				
			loop i from: 0 to:length(positions)-2{
				draw circle(0.03,positions[i]) color: ped_color;
				/*
				if((i+1) < length(positions)){
					geometry trail_line <- line([positions[i],positions[i+1]]);
					rgb line_color <- get_speed_color(positions[i] distance_to positions[i+1]); 
					draw trail_line color: line_color width:100;
				}*/
			}
//			draw line([positions[length(positions)-1],location]) color: color width:100; 
		}
		
		draw triangle(shoulder_length-0.2, 0.20) color: ped_color border: #black rotate: 90 + heading;
		draw ellipse(shoulder_length, 0.15) color: ped_color border: #black rotate: 90 + heading;
		draw circle(0.075) color: ped_color border: #black rotate: 90 + heading;
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
	
	int id;
	
	point exit_ext1;
	point exit_ext2;
	
	float exit_width;
	float weight;
	
	list<point> exit_points <- [];
	point centroid <- centroid(shape);

	aspect default {
		draw shape color: #red border: #black;
		draw ""+id color: #white;
	}
}


experiment main1 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 1;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment main2 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 2;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment main3 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 3;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment main4 type: gui {
	float minimum_cycle_duration <- cycle_duration;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area_stage.shp");
	
	parameter "group_size" var: group_size_avg <- 4;
	
	output {
		display map background: background_color{
			//image file:"../includes/municipio/municipio.png" transparency: 0.5;
			species obstacle;
			species people;
			species target;
			species group;
		}
		display chart_display refresh: every(100 #cycles) {
			chart "Percentage of Evacuated pedestrians" type: series {
				data "Remaining pedestrians" value: nb_remaining color: #green;
			}
		}
	}
}

experiment main5 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 5;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment main6 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 6;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment main7 type: gui {
	float minimum_cycle_duration <- 0.05;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area.shp");
	
	parameter "group_size" var: group_size_avg <- 7;
	
	output {
		display map{
			species obstacle;
			species people;
			species target;
			species group;
		}
	}
}

experiment z_pombal type: gui {
	float minimum_cycle_duration <- cycle_duration;
	parameter "buildings" var: buildings_file <- file("../includes/pombal/buildings2.shp");
	parameter "targets" var: targets_file <- file("../includes/pombal/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/pombal/start_area.shp");
	
	parameter "egress_interval_period" var: egress_interval_period <- 0.0001 #hours;
	
	output {
		display map background: background_color{
			species obstacle;
			species people;
			species target;
			species group;
		}
		display chart_display refresh: every(100 #cycles) {
			chart "Percentage of Evacuated pedestrians" type: series {
				data "Remaining pedestrians" value: nb_remaining color: #green;
			}
		}
	}
}

experiment z_alameda type: gui {
	float minimum_cycle_duration <- cycle_duration;
	parameter "buildings" var: buildings_file <- file("../includes/alameda/buildings3.shp");
	parameter "targets" var: targets_file <- file("../includes/alameda/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/alameda/start_area.shp");
	
	parameter "egress_interval_period" var: egress_interval_period <- 0.0001 #hours;
	
	output {
		display map background: background_color{
			species obstacle;
			species people;
			species target;
			species group;
		}
		display chart_display refresh: every(100 #cycles) {
			chart "Percentage of Evacuated pedestrians" type: series {
				data "Remaining pedestrians" value: nb_remaining color: #green;
			}
		}
	}
}

experiment z_comercio type: gui {
	float minimum_cycle_duration <- cycle_duration;
	parameter "buildings" var: buildings_file <- file("../includes/comercio/comercio1.shp");
	parameter "targets" var: targets_file <- file("../includes/comercio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/comercio/praca.shp");
	
	parameter "egress_interval_period" var: egress_interval_period <- 0.0001 #hours;
	
	output {
		display map background: background_color{
			species obstacle;
			species people;
			species target;
			species group;
		}
		display chart_display refresh: every(100 #cycles) {
			chart "Percentage of Evacuated pedestrians" type: series {
				data "Remaining pedestrians" value: nb_remaining color: #green;
			}
		}
	}
}

experiment monumental type: gui {
	float minimum_cycle_duration <- cycle_duration;
	parameter "buildings" var: buildings_file <- file("../includes/municipio/buildings.shp");
	parameter "targets" var: targets_file <- file("../includes/municipio/targets.shp");
	parameter "start_area" var: praca_file <- file("../includes/municipio/start_area_stage.shp");
	
	parameter "egress_interval_period" var: egress_interval_period <- 0.00001 #hours;
	
	output {
		display map background: background_color{
			image file:"../includes/municipio/municipio.png" transparency: 0.3;
			species obstacle;
			species people;
			species target;
			species group;
		}
		display chart_display refresh: every(5 #minutes) {
			chart "Percentage of Evacuated pedestrians" type: series {
				data "Remaining pedestrians" value: nb_remaining color: #green;
			}
		}
	}
}
