#include <math.h>
#include <string.h>
#include <gsl/gsl_matrix.h>
#include <laser_scan_matcher/laser_scan_matcher.h>

#include <icp/gpc.h>
#include <icp/egsl.h>

#include <csm/csm_all.h>
#include <csm/laser_data.h>

INLINE double mysin(double x) {
	const double a = -1.0/6.0;
	const double b = +1.0/120.0;
	double x2 = x*x;
	return x * (.99 + x2 * ( a + b * x2));
}

#define DEBUG_SEARCH(a) ;

extern int distance_counter;
INLINE double local_distance_squared_d(const double* a, const double* b)  {
	distance_counter++;
	double x = a[0] - b[0];
	double y = a[1] - b[1];
	return x*x + y*y;
}

#define SQUARE(a) ((a)*(a))

int compatible(struct sm_params*params, int i, int j) {
	if(!params->do_alpha_test) return 1;

	double theta0 = 0; /* FIXME */
	if((params->laser_sens->alpha_valid[i]==0) ||
	 (params->laser_ref->alpha_valid[j]==0))
	 return 1;

	double alpha_i = params->laser_sens->alpha[i];
	double alpha_j = params->laser_ref->alpha[j];
	double tolerance = deg2rad(params->do_alpha_test_thresholdDeg);

	/** FIXME remove alpha test */
	double theta = angleDiff(alpha_j, alpha_i);
	if(fabs(angleDiff(theta,theta0))>
		tolerance+deg2rad(params->max_angular_correction_deg)) {
	 return 0;
	} else {
	 return 1;
	}
}

val compute_C_k(val p_j1, val p_j2)  {	
	val d = egsl_sub(p_j1, p_j2);
	double alpha = M_PI/2 + atan2( egsl_atv(d,1), egsl_atv(d,0));
	double c = cos(alpha); double s = sin(alpha);
	double m[2*2] = {
		c*c, c*s,
		c*s, s*s
	};
	return egsl_vFda(2,2,m);
}

val dC_drho(val p1, val p2) {
	double eps = 0.001;

	val C_k = compute_C_k(p1, p2);	
	val p1b = egsl_sum(p1, egsl_scale(eps/egsl_norm(p1),p1));
	val C_k_eps = compute_C_k(p1b,p2);
	return egsl_scale(1/eps, egsl_sub(C_k_eps,C_k));
}

void swap_double(double*a,double*b) {
	double t = *a; *a = *b; *b=t;
}

void quicksort(double *array, int begin, int end) {
	if (end > begin) {
		double pivot = array[begin];
		int l = begin + 1;
		int r = end+1;
		while(l < r) {
			if (array[l] < pivot) {
				l++;
			} else {
				r--;
				swap_double(array+l, array+r); 
			}
		}
		l--;
		swap_double(array+begin, array+l);
		if(l>begin)
		quicksort(array, begin, l);
		if(end>r)
		quicksort(array, r, end);
	}
}

int _compute_next_estimate(struct sm_params*params, const double x_old[3], double x_new[3]);
int _termination_criterion(struct sm_params*params, const double*delta);
void _find_correspondences(struct sm_params*params);
void _find_correspondences_tricks(struct sm_params*params);
void _debug_correspondences(struct sm_params * params);
void _kill_outliers_trim(struct sm_params*params, double*total_error);
void _kill_outliers_double(struct sm_params*params);
void _compute_covariance_exact(LDP laser_ref, LDP laser_sens, const gsl_vector*x, val *cov0_x, val *dx_dy1, val *dx_dy2);
void _visibilityTest(LDP ld, const gsl_vector*x_old);
void _ld_invalid_if_outside(LDP ld, double min_reading, double max_reading);

int _compute_next_estimate(struct sm_params*params, 
	const double x_old[3], double x_new[3]) 
{
	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;
	
	struct gpc_corr c[laser_sens->nrays];

	int i; int k=0;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!laser_sens->valid[i])
			continue;
			
		if(!ld_valid_corr(laser_sens,i))
			continue;
		
		int j1 = laser_sens->corr[i].j1;
		int j2 = laser_sens->corr[i].j2;

		c[k].valid = 1;
		
		if(laser_sens->corr[i].type == correspondence::corr_pl) {

			c[k].p[0] = laser_sens->points[i].p[0];
			c[k].p[1] = laser_sens->points[i].p[1];
			c[k].q[0] = laser_ref->points[j1].p[0];
			c[k].q[1] = laser_ref->points[j1].p[1];

			/** TODO: here we could use the estimated alpha */
			double diff[2];
			diff[0] = laser_ref->points[j1].p[0]-laser_ref->points[j2].p[0];
			diff[1] = laser_ref->points[j1].p[1]-laser_ref->points[j2].p[1];
			double one_on_norm = 1 / sqrt(diff[0]*diff[0]+diff[1]*diff[1]);
			double normal[2];
			normal[0] = +diff[1] * one_on_norm;
			normal[1] = -diff[0] * one_on_norm;

			double cos_alpha = normal[0];
			double sin_alpha = normal[1];
						
			c[k].C[0][0] = cos_alpha*cos_alpha;
			c[k].C[1][0] = 
			c[k].C[0][1] = cos_alpha*sin_alpha;
			c[k].C[1][1] = sin_alpha*sin_alpha;
			
			/*  sm_debug("k=%d, i=%d sens_phi: %fdeg, j1=%d j2=%d,  alpha_seg=%f, cos=%f sin=%f \n", k,i,
				rad2deg(laser_sens->theta[i]), j1,j2, atan2(sin_alpha,cos_alpha), cos_alpha,sin_alpha);*/
			
#if 0
			/* Note: it seems that because of numerical errors this matrix might be
			   not semidef positive. */
			double det = c[k].C[0][0] * c[k].C[1][1] - c[k].C[0][1] * c[k].C[1][0];
			double trace = c[k].C[0][0] + c[k].C[1][1];
			
			int semidef = (det >= 0) && (trace>0);
			if(!semidef) {
				// printf("%d: Adjusting correspondence weights\n",i);
				double eps = -det;
				c[k].C[0][0] += 2*sqrt(eps);
				c[k].C[1][1] += 2*sqrt(eps);
			}
#endif			
		} else {
			c[k].p[0] = laser_sens->points[i].p[0];
			c[k].p[1] = laser_sens->points[i].p[1];
			
			projection_on_segment_d(
				laser_ref->points[j1].p,
				laser_ref->points[j2].p,
				laser_sens->points_w[i].p,
				c[k].q);

			/* Identity matrix */
			c[k].C[0][0] = 1;
			c[k].C[1][0] = 0;
			c[k].C[0][1] = 0;
			c[k].C[1][1] = 1;
		}
		
		
		double factor = 1;
		
		/* Scale the correspondence weight by a factor concerning the 
		   information in this reading. */
		if(params->use_ml_weights) {
			int have_alpha = 0;
			double alpha = 0;
			if(!is_nan(laser_ref->true_alpha[j1])) {
				alpha = laser_ref->true_alpha[j1];
				have_alpha = 1;
			} else if(laser_ref->alpha_valid[j1]) {
				alpha = laser_ref->alpha[j1];;
				have_alpha = 1;
			} else have_alpha = 0;
			
			if(have_alpha) {
				double pose_theta = x_old[2];
				/** Incidence of the ray 
					Note that alpha is relative to the first scan (not the world)
					and that pose_theta is the angle of the second scan with 
					respect to the first, hence it's ok. */
				double beta = alpha - (pose_theta + laser_sens->theta[i]);
				factor = 1 / square(cos(beta));
			} else {
				static int warned_before = 0;
				if(!warned_before) {
					sm_error("Param use_ml_weights was active, but not valid alpha[] or true_alpha[]." 
					          "Perhaps, if this is a single ray not having alpha, you should mark it as inactive.\n");						
					sm_error("Writing laser_ref: \n");						
					ld_write_as_json(laser_ref, stderr);
					warned_before = 1;
				}
			}
		} 
		
		/* Weight the points by the sigma in laser_sens */
		if(params->use_sigma_weights) {
			if(!is_nan(laser_sens->readings_sigma[i])) {
				factor *= 1 / square(laser_sens->readings_sigma[i]);
			} else {
				static int warned_before = 0;
				if(!warned_before) {
					sm_error("Param use_sigma_weights was active, but the field readings_sigma[] was not filled in.\n");						
					sm_error("Writing laser_sens: \n");						
					ld_write_as_json(laser_sens, stderr);
				}
			}
		}
		
		c[k].C[0][0] *= factor;
		c[k].C[1][0] *= factor;
		c[k].C[0][1] *= factor;
		c[k].C[1][1] *= factor;
		
		k++;
	}
	
	/* TODO: use prior for odometry */
	double std = 0.11;
	const double inv_cov_x0[9] = 
		{1/(std*std), 0, 0,
		 0, 1/(std*std), 0,
		 0, 0, 0};
	
	
	int ok = gpc_solve(k, c, 0, inv_cov_x0, x_new);
	if(!ok) {
		sm_error("gpc_solve_valid failed\n");
		return 0;
	}

	double old_error = gpc_total_error(c, k, x_old);
	double new_error = gpc_total_error(c, k, x_new);

	sm_debug("\tcompute_next_estimate: old error: %f  x_old= %s \n", old_error, friendly_pose(x_old));
	sm_debug("\tcompute_next_estimate: new error: %f  x_new= %s \n", new_error, friendly_pose(x_new));
	sm_debug("\tcompute_next_estimate: new error - old_error: %g \n", new_error-old_error);

	double epsilon = 0.000001;
	if(new_error > old_error + epsilon) {
		sm_error("\tcompute_next_estimate: something's fishy here! Old error: %lf  new error: %lf  x_old %lf %lf %lf x_new %lf %lf %lf\n",old_error,new_error,x_old[0],x_old[1],x_old[2],x_new[0],x_new[1],x_new[2]);
	}
	
	return 1;
}

int _termination_criterion(struct sm_params*params, const double*delta){
	double a = norm_d(delta);
	double b = fabs(delta[2]);
	return (a<params->epsilon_xy) && (b<params->epsilon_theta);
}

void _find_correspondences(struct sm_params*params) {
	const LDP laser_ref  = params->laser_ref;
	const LDP laser_sens = params->laser_sens;

	int i;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_ray(laser_sens,i)) {
			// sm_debug("dumb: i %d is invalid \n", i);
			ld_set_null_correspondence(laser_sens, i);
			continue; 
		}

		double *p_i_w = laser_sens->points_w[i].p;
		
		int j1 = -1;
		double best_dist = 10000;
		
		int from; int to; int start_cell;
		possible_interval(p_i_w, laser_ref, params->max_angular_correction_deg,
			params->max_linear_correction, &from, &to, &start_cell);

		// sm_debug("dumb: i %d from  %d to %d \n", i, from, to);
		int j;
		for(j=from;j<=to;j++) {
			if(!ld_valid_ray(laser_ref,j)) {
				// sm_debug("dumb: i %d      j %d invalid\n", i, j);
				continue;
			}
			double dist = distance_squared_d(p_i_w, laser_ref->points[j].p);
			// sm_debug("dumb: i %d j1 %d j %d d %f\n", i,j1,j,dist);
			if(dist>square(params->max_correspondence_dist)) continue;
			
			if( (-1 == j1) || (dist < best_dist) ) {
				if(compatible(params, i, j)) {
					j1 = j; 
					best_dist = dist;
				}
			} 
		}
		
		if(j1==-1) {/* no match */
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}
		/* Do not match with extrema*/
		if(j1==0 || (j1 == (laser_ref->nrays-1))) {/* no match */
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}
		
		int j2;
		int j2up   = ld_next_valid_up   (laser_ref, j1);
		int j2down = ld_next_valid_down (laser_ref, j1);
		if((j2up==-1)&&(j2down==-1)) {
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}
		if(j2up  ==-1) { j2 = j2down; } else
		if(j2down==-1) { j2 = j2up; } else {
			double dist_up   = distance_squared_d(p_i_w, laser_ref->points[j2up  ].p);
			double dist_down = distance_squared_d(p_i_w, laser_ref->points[j2down].p);
			j2 = dist_up < dist_down ? j2up : j2down;
		}
		
		ld_set_correspondence(laser_sens, i, j1, j2);
		laser_sens->corr[i].dist2_j1 = best_dist;
		laser_sens->corr[i].type = 
			params->use_point_to_line_distance ? correspondence::corr_pl : correspondence::corr_pp;
		
	}
	
}

void _find_correspondences_tricks(struct sm_params*params) {
	const LDP laser_ref  = params->laser_ref;
	const LDP laser_sens = params->laser_sens;
	int i;
	
	/* Handy constant */
	double C1 =  (double)laser_ref->nrays / (laser_ref->max_theta-laser_ref->min_theta) ;
	double max_correspondence_dist2 = square(params->max_correspondence_dist);
	/* Last match */
	int last_best = -1;
	const point2d * restrict points_w = laser_sens->points_w;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_ray(laser_sens,i)) {
			ld_set_null_correspondence(laser_sens,i);
			continue; 
		}
		
		const double *p_i_w = points_w[i].p;
		double p_i_w_nrm2 = points_w[i].rho;
		double p_i_w_phi = points_w[i].phi;
		
		/** Search domain for j1 */
		int from = 0; 
		int to = laser_ref->nrays-1; 
		
		int start_cell = (int) ((p_i_w_phi - laser_ref->min_theta) * C1); 

		/** Current best match */
		int j1 = -1;
		/** and his distance */
		double best_dist = 42;
		
		/** If last match was succesful, then start at that index + 1 */
		int we_start_at; 
		if (last_best==-1)
			we_start_at = start_cell;
	 	else
			we_start_at = last_best + 1;
		
		if(we_start_at > to) we_start_at = to;
		if(we_start_at < from) we_start_at = from;
		
		int up =  we_start_at+1; 
		int down = we_start_at; 
		double last_dist_up = 0; /* first is down */
		double last_dist_down = -1;	

		int up_stopped = 0; 
		int down_stopped = 0;
	
		DEBUG_SEARCH(printf("i=%d p_i_w = %f %f\n",i, p_i_w[0], p_i_w[1]));
		DEBUG_SEARCH(printf("i=%d [from %d down %d mid %d up %d to %d]\n",
			i,from,down,start_cell,up,to));
		
		while ( (!up_stopped) || (!down_stopped) ) {
			int now_up = up_stopped ? 0 : 
			           down_stopped ? 1 : last_dist_up < last_dist_down;
			DEBUG_SEARCH(printf("|"));

			/* Now two symmetric chunks of code, the now_up and the !now_up */
			if(now_up) {
				DEBUG_SEARCH(printf("up %d ",up));
				/* If we have crossed the "to" boundary we stop searching
					on the "up" direction. */
				if(up > to) { 
					up_stopped = 1; continue; }
				/* Just ignore invalid rays */
				if(!laser_ref->valid[up]) { 
					++up; continue; }
				
				/* This is the distance from p_i_w to the "up" point*/
				last_dist_up = local_distance_squared_d(p_i_w, laser_ref->points[up].p);
				
				/* If it is less than the best point, it is our new j1 */
				if((last_dist_up < best_dist) || (j1==-1) )
						j1 = up, best_dist = last_dist_up;
				
				/* If we are moving away from start_cell */
				if (up > start_cell) {
					double delta_theta = (laser_ref->theta[up] - p_i_w_phi);
					/* We can compute a bound for early stopping. Currently
					   our best point has distance best_dist; we can compute
					   min_dist_up, which is the minimum distance that can have
					   points for j>up (see figure)*/
					double min_dist_up = p_i_w_nrm2 * 
						((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));
					/* If going up we can't make better than best_dist, then
					    we stop searching in the "up" direction */
					if(SQUARE(min_dist_up) > best_dist) { 
						up_stopped = 1; continue;
					}
					/* If we are moving away, then we can implement the jump tables
					   optimizations. */
					up += 
						/* If p_i_w is shorter than "up" */
						(laser_ref->readings[up] < p_i_w_nrm2) 
						?
						/* We can jump to a bigger point */
						laser_ref->up_bigger[up] 
						/* Or else we jump to a smaller point */ 
						: laser_ref->up_smaller[up];
						
				} else 
					/* If we are moving towards "start_cell", we can't do any
					   ot the previous optimizations and we just move to the next point */
					++up;
			}
			
			/* This is the specular part of the previous chunk of code. */
			if(!now_up) {
				DEBUG_SEARCH(printf("down %d ",down));
				if(down < from) { 
					down_stopped = 1; continue; }
				if(!laser_ref->valid[down]) { 
					--down; continue; }
		
				last_dist_down = local_distance_squared_d(p_i_w, laser_ref->points[down].p);
				if( (last_dist_down < best_dist) || (j1==-1) )
						j1 = down, best_dist = last_dist_down;

				if (down < start_cell) {
					double delta_theta = (p_i_w_phi - laser_ref->theta[down]);
					double min_dist_down = p_i_w_nrm2 * 
						((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));
					if( SQUARE(min_dist_down) > best_dist) { 
						down_stopped = 1; continue;
					}
					down += (laser_ref->readings[down] < p_i_w_nrm2) ?
						laser_ref->down_bigger[down] : laser_ref->down_smaller[down];
				} else --down;
			}
			
		}
		
		DEBUG_SEARCH(printf("i=%d j1=%d dist=%f\n",i,j1,best_dist));
		
		/* If no point matched. */
		if( (-1==j1) || (best_dist > max_correspondence_dist2) ) {
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}
		/* We ignore matching the first or the last point in the scan */
		if( 0==j1 || j1 == (laser_ref->nrays-1)) {/* no match */
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}

		/* Now we want to find j2, the second best match. */
		int j2;
		/* We find the next valid point, up and down */
		int j2up   = ld_next_valid_up   (laser_ref, j1);
		int j2down = ld_next_valid_down (laser_ref, j1);
		/* And then (very boring) we use the nearest */
		if((j2up==-1)&&(j2down==-1)) {
			ld_set_null_correspondence(laser_sens, i);
			continue;
		}
		if(j2up  ==-1) { j2 = j2down; } else
		if(j2down==-1) { j2 = j2up; } else {
			double dist_up   = local_distance_squared_d(p_i_w, laser_ref->points[j2up  ].p);
			double dist_down = local_distance_squared_d(p_i_w, laser_ref->points[j2down].p);
			j2 = dist_up < dist_down ? j2up : j2down;
		}

		last_best = j1;
		
		laser_sens->corr[i].valid = 1;
		laser_sens->corr[i].j1 = j1;
		laser_sens->corr[i].j2 = j2;
		laser_sens->corr[i].dist2_j1 = best_dist;
		laser_sens->corr[i].type = 
			params->use_point_to_line_distance ? correspondence::corr_pl : correspondence::corr_pp;
		
	}
}

void _debug_correspondences(struct sm_params * params) {
	LDP laser_sens = params->laser_sens;
	/** Do the test */
	_find_correspondences_tricks(params);
	struct correspondence c1[laser_sens->nrays];
	struct correspondence * c2 = laser_sens->corr;
	memcpy(c1, c2, sizeof(struct correspondence) * laser_sens->nrays);
	long hash1 = ld_corr_hash(laser_sens);
	_find_correspondences(params);
	long hash2 = ld_corr_hash(laser_sens);
	if(hash1 != hash2) {
		sm_error("find_correspondences_tricks might be buggy\n");
		int i = 0; for(i=0;i<laser_sens->nrays;i++) {
			if( (c1[i].valid != c2[i].valid) ||
				(c1[i].j1 != c2[i].j1) || (c1[i].j2 != c2[i].j2) ) {
					sm_error("\t   tricks: c1[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
						i, c1[i].valid, c1[i].j1, c1[i].j2, c1[i].dist2_j1);
					sm_error("\tno tricks: c2[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
						i, c2[i].valid, c2[i].j1, c2[i].j2, c2[i].dist2_j1);
				}
		}
		if(1) exit(-1);
	}
}

void _kill_outliers_trim(struct sm_params*params,  double*total_error) {
		
	if(JJ) jj_context_enter("kill_outliers_trim");
		
	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;
	
	/* dist2, indexed by k, contains the error for the k-th correspondence */
	int k = 0; 
	double dist2[laser_sens->nrays];
		
	int i;
	double dist[laser_sens->nrays];
	/* for each point in laser_sens */
	for(i=0;i<laser_sens->nrays;i++) {
		/* which has a valid correspondence */
		if(!ld_valid_corr(laser_sens, i)) { dist[i]=NAN; continue; }
		double *p_i_w = laser_sens->points_w[i].p;
		
		int j1 = laser_sens->corr[i].j1;
		int j2 = laser_sens->corr[i].j2;
		/* Compute the distance to the corresponding segment */
		dist[i]=  dist_to_segment_d(
			laser_ref->points[j1].p, laser_ref->points[j2].p, p_i_w);
		dist2[k] = dist[i];
		k++;	
	}
	
	
	if(JJ) jj_add_int("num_valid_before", k);
	if(JJ) jj_add_double_array("dist_points", dist2, laser_sens->nrays);
	if(JJ) jj_add_double_array("dist_corr_unsorted", dist2, k);

#if 0	
	double dist2_copy[k]; for(i=0;i<k;i++) dist2_copy[i] = dist2[i];
#endif 

	/* two errors limits are defined: */
		/* In any case, we don't want more than outliers_maxPerc% */
		int order = (int)floor(k*(params->outliers_maxPerc));
			order = GSL_MAX(0, GSL_MIN(order, k-1));

	/* The dists for the correspondence are sorted
	   in ascending order */
		quicksort(dist2, 0, k-1);
		double error_limit1 = dist2[order];
		if(JJ) jj_add_double_array("dist_corr_sorted", dist2, k);
	
		/* Then we take a order statics (o*K) */
		/* And we say that the error must be less than alpha*dist(o*K) */
		int order2 = (int)floor(k*params->outliers_adaptive_order);
			order2 = GSL_MAX(0, GSL_MIN(order2, k-1));
		double error_limit2 = params->outliers_adaptive_mult*dist2[order2];
	
	double error_limit = GSL_MIN(error_limit1, error_limit2);
	
#if 0
	double error_limit1_ho = hoare_selection(dist2_copy, 0, k-1, order);
	double error_limit2_ho = error_limit2;
	if((error_limit1_ho != error_limit1) || (error_limit2_ho != error_limit2)) {
		printf("%f == %f    %f  == %f\n",
			error_limit1_ho, error_limit1, error_limit2_ho, error_limit2);
	}
#endif

	if(JJ) jj_add_double_array("dist_corr_sorted", dist2, k);
	if(JJ) jj_add_double("error_limit_max_perc", error_limit1);
	if(JJ) jj_add_double("error_limit_adaptive", error_limit2);
	if(JJ) jj_add_double("error_limit", error_limit);
	
	sm_debug("\ticp_outliers: maxPerc %f error_limit: fix %f adaptive %f \n",
		params->outliers_maxPerc,error_limit1,error_limit2);

	*total_error = 0;
	int nvalid = 0;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_corr(laser_sens, i)) continue;
		if(dist[i] > error_limit) {
			laser_sens->corr[i].valid = 0;
			laser_sens->corr[i].j1 = -1;
			laser_sens->corr[i].j2 = -1;
		} else {
			nvalid++;
			*total_error += dist[i];
		}
	}
	
	sm_debug("\ticp_outliers: valid %d/%d (limit: %f) mean error = %f \n",nvalid,k,error_limit,
		*total_error/nvalid);	

	if(JJ) jj_add_int("num_valid_after", nvalid);
	if(JJ) jj_add_double("total_error", *total_error);
	if(JJ) jj_add_double("mean_error", *total_error / nvalid);
		
	if(JJ) jj_context_exit();
}

void _kill_outliers_double(struct sm_params*params) {
	double threshold = 3; /* TODO: add as configurable */

	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;

	double dist2_i[laser_sens->nrays];
	double dist2_j[laser_ref->nrays];
	int j; for(j=0;j<laser_ref->nrays;j++) 
		dist2_j[j]= 1000000;
	
	int i;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_corr(laser_sens, i)) continue;
		int j1 = laser_sens->corr[i].j1;
		dist2_i[i] = laser_sens->corr[i].dist2_j1;
		dist2_j[j1] = GSL_MIN(dist2_j[j1], dist2_i[i]);
	}
	
	int nkilled = 0;
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_corr(laser_sens, i)) continue;
		int j1 = laser_sens->corr[i].j1;
		if(dist2_i[i] > (threshold*threshold)*dist2_j[j1]) {
			laser_sens->corr[i].valid=0;
			nkilled ++;
		}
	}
	sm_debug("\tkill_outliers_double: killed %d correspondences\n",nkilled);
}

void _compute_covariance_exact(
	LDP laser_ref, LDP laser_sens, const gsl_vector*x,
		val *cov0_x, val *dx_dy1, val *dx_dy2)
{
	egsl_push_named("compute_covariance_exact");
	
	val d2J_dxdy1 = egsl_zeros(3,(size_t)laser_ref ->nrays);
	val d2J_dxdy2 = egsl_zeros(3,(size_t)laser_sens->nrays);
	
	/* the three pieces of d2J_dx2 */
	val d2J_dt2       = egsl_zeros(2,2);
	val d2J_dt_dtheta = egsl_zeros(2,1);
	val d2J_dtheta2   = egsl_zeros(1,1);
	
	double theta = x->data[2];
	val t = egsl_vFa(2,x->data);
	
	int i; 
	for(i=0;i<laser_sens->nrays;i++) {
		if(!ld_valid_corr(laser_sens,i)) continue;
		egsl_push_named("compute_covariance_exact iteration");

		int j1 = laser_sens->corr[i].j1;
		int j2 = laser_sens->corr[i].j2;

		val p_i  = egsl_vFa(2, laser_sens->points[i].p);
		val p_j1 = egsl_vFa(2, laser_ref ->points[j1].p);
		val p_j2 = egsl_vFa(2, laser_ref ->points[j2].p);
		
		/* v1 := egsl_rot(theta+M_PI/2)*p_i */
		val v1 = egsl_mult(egsl_rot(theta+M_PI/2), p_i);		
		/* v2 := (egsl_rot(theta)*p_i+t-p_j1) */
		val v2 = egsl_sum(egsl_mult(egsl_rot(theta),p_i),egsl_sum(t,egsl_scale(-1.0,p_j1)));
		/* v3 := egsl_rot(theta)*v_i */
		val v3 = egsl_vers(theta + laser_sens->theta[i]);
		/* v4 := egsl_rot(theta+PI/2)*v_i; */
		val v4 = egsl_vers(theta + laser_sens->theta[i] + M_PI/2);
		
		val C_k = compute_C_k(p_j1, p_j2);
		
		val d2J_dt2_k = egsl_scale(2.0, C_k);
		val d2J_dt_dtheta_k = egsl_scale(2.0,egsl_mult(C_k,v1));
		
		val v_new = egsl_mult(egsl_rot(theta+M_PI), p_i);
		val d2J_dtheta2_k = egsl_scale(2.0, egsl_sum( egsl_mult(egsl_transpose(v2),egsl_mult(C_k,v_new)), egsl_mult(egsl_transpose(v1),egsl_mult(C_k,v1))));
		egsl_add_to(d2J_dt2, d2J_dt2_k);
		egsl_add_to(d2J_dt_dtheta, d2J_dt_dtheta_k ); 
		egsl_add_to(d2J_dtheta2, d2J_dtheta2_k);
		
		/* for measurement rho_i  in the second scan */
		val d2Jk_dtdrho_i = egsl_scale(2.0, egsl_mult(C_k,v3)); 
		val d2Jk_dtheta_drho_i = egsl_scale(2.0, egsl_sum( egsl_mult(egsl_transpose(v2),egsl_mult(C_k,v4)),  egsl_mult(egsl_transpose(v3),egsl_mult(C_k,v1))));
 		egsl_add_to_col(d2J_dxdy2, (size_t)i, egsl_compose_col(d2Jk_dtdrho_i, d2Jk_dtheta_drho_i));
		
		/* for measurements rho_j1, rho_j2 in the first scan */
		
		val dC_drho_j1 = dC_drho(p_j1, p_j2);
		val dC_drho_j2 = dC_drho(p_j2, p_j1);
	
		
		val v_j1 = egsl_vers(laser_ref->theta[j1]);
		
		val d2Jk_dt_drho_j1 = egsl_sum(egsl_scale(-2.0,egsl_mult(C_k,v_j1)), egsl_scale(2.0,egsl_mult(dC_drho_j1,v2)));
		val d2Jk_dtheta_drho_j1 = egsl_sum( egsl_scale(-2.0, egsl_mult(egsl_transpose(v_j1),egsl_mult(C_k,v1))), egsl_mult(egsl_transpose(v2),egsl_mult(dC_drho_j1,v1)));
		egsl_add_to_col(d2J_dxdy1, (size_t)j1, egsl_compose_col(d2Jk_dt_drho_j1, d2Jk_dtheta_drho_j1));
		
		/* for measurement rho_j2*/
		val d2Jk_dt_drho_j2 = egsl_scale(2.0, egsl_mult( dC_drho_j2,v2));
		val d2Jk_dtheta_drho_j2 = egsl_scale(2.0, egsl_mult(egsl_transpose(v2),egsl_mult(dC_drho_j2,v1)));
		egsl_add_to_col(d2J_dxdy1, (size_t)j2, egsl_compose_col(d2Jk_dt_drho_j2, d2Jk_dtheta_drho_j2));

		egsl_pop_named("compute_covariance_exact iteration");
	}

	/* composes matrix  d2J_dx2  from the pieces*/
	val d2J_dx2   = egsl_compose_col( egsl_compose_row(    d2J_dt2      ,   d2J_dt_dtheta),
	                          egsl_compose_row(egsl_transpose(d2J_dt_dtheta),     d2J_dtheta2));
	
	val edx_dy1 = egsl_scale(-1.0, egsl_mult(egsl_inverse(d2J_dx2), d2J_dxdy1));
	val edx_dy2 = egsl_scale(-1.0, egsl_mult(egsl_inverse(d2J_dx2), d2J_dxdy2));

	val ecov0_x = egsl_sum(egsl_mult(edx_dy1,egsl_transpose(edx_dy1)),egsl_mult(edx_dy2,egsl_transpose(edx_dy2)) );

	/* With the egsl_promote we save the matrix in the previous
	   context */
	*cov0_x = egsl_promote(ecov0_x);
	*dx_dy1 = egsl_promote(edx_dy1);
	*dx_dy2 = egsl_promote(edx_dy2);
	
	egsl_pop_named("compute_covariance_exact");	
	/* now edx_dy1 is not valid anymore, but *dx_dy1 is. */
}

void _visibilityTest(LDP laser_ref, const gsl_vector*u) {

	double theta_from_u[laser_ref->nrays];
	
	int j;
	for(j=0;j<laser_ref->nrays;j++) {
		if(!ld_valid_ray(laser_ref,j)) continue;
		theta_from_u[j] = 
			atan2(gvg(u,1)-laser_ref->points[j].p[1],
			      gvg(u,0)-laser_ref->points[j].p[0]);
	}
	
	sm_debug("\tvisibility: Found outliers: ");
	int invalid = 0;
	for(j=1;j<laser_ref->nrays;j++) {
		if(!ld_valid_ray(laser_ref,j)||!ld_valid_ray(laser_ref,j-1)) continue;
		if(theta_from_u[j]<theta_from_u[j-1]) {
			laser_ref->valid[j] = 0;
			invalid ++;
			sm_debug("%d ",j);
		}
	}
	sm_debug("\n");
}

void _ld_invalid_if_outside(LDP ld, double min_reading, double max_reading) {
	int i;
	for(i=0;i<ld->nrays;i++) {
		if(!ld_valid_ray(ld, i)) continue;
		double r = ld->readings[i];
		if( r <= min_reading || r > max_reading)
			ld->valid[i] = 0;
	}
}

namespace scan_tools
{

int LaserScanMatcher::_icp_loop(struct sm_params*params, const double*q0, double*x_new, 
	double*total_error, int*valid, int*iterations) {
	if(any_nan(q0,3)) {
		sm_error("icp_loop: Initial pose contains nan: %s\n", friendly_pose(q0));
		return 0;
	}
		
		
	LDP laser_sens = params->laser_sens;
	double x_old[3], delta[3], delta_old[3] = {0,0,0};
	copy_d(q0, 3, x_old);
	unsigned int hashes[params->max_iterations];
	int iteration;
	
	sm_debug("icp: starting at  q0 =  %s  \n", friendly_pose(x_old));
	
	if(JJ) jj_loop_enter("iterations");
	
	int all_is_okay = 1;
	
	for(iteration=0; iteration<params->max_iterations;iteration++) {
		if(JJ) jj_loop_iteration();
		if(JJ) jj_add_double_array("x_old", x_old, 3);

		egsl_push_named("icp_loop iteration");
		sm_debug("== icp_loop: starting iteration. %d  \n", iteration);

		/** Compute laser_sens's points in laser_ref's coordinates
		    by roto-translating by x_old */
		ld_compute_world_coords(laser_sens, x_old);

		/** Find correspondences (the naif or smart way) */
		if(params->use_corr_tricks)
			_find_correspondences_tricks(params);
		else
			_find_correspondences(params);

		/** If debug_verify_tricks, make sure that find_correspondences_tricks()
		    and find_correspondences() return the same answer */
			if(params->debug_verify_tricks)
				_debug_correspondences(params);

		/* If not many correspondences, bail out */
		int num_corr = ld_num_valid_correspondences(laser_sens);
		double fail_perc = min_perc_correspondence_;
		if(num_corr < fail_perc * laser_sens->nrays) { /* TODO: arbitrary */
			sm_error("	: before trimming, only %d correspondences.\n",num_corr);
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		if(JJ) jj_add("corr0", corr_to_json(laser_sens->corr, laser_sens->nrays));

		/* Kill some correspondences (using dubious algorithm) */
		if(params->outliers_remove_doubles)
			_kill_outliers_double(params);
		
		int num_corr2 = ld_num_valid_correspondences(laser_sens);

		if(JJ) jj_add("corr1", corr_to_json(laser_sens->corr, laser_sens->nrays));
		
		double error=0;
		/* Trim correspondences */
		_kill_outliers_trim(params, &error);
		int num_corr_after = ld_num_valid_correspondences(laser_sens);
		
		if(JJ) {
			jj_add("corr2", corr_to_json(laser_sens->corr, laser_sens->nrays));
			jj_add_int("num_corr0", num_corr);
			jj_add_int("num_corr1", num_corr2);
			jj_add_int("num_corr2", num_corr_after);
		}

		*total_error = error; 
		*valid = num_corr_after;

		sm_debug("  icp_loop: total error: %f  valid %d   mean = %f\n", *total_error, *valid, *total_error/ *valid);
		
		/* If not many correspondences, bail out */
		if(num_corr_after < fail_perc * laser_sens->nrays){
			sm_error("  icp_loop: failed: after trimming, only %d correspondences.\n",num_corr_after);
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		/* Compute next estimate based on the correspondences */
		if(!_compute_next_estimate(params, x_old, x_new)) {
			sm_error("  icp_loop: Cannot compute next estimate.\n");
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration");
			break;			
		}

		pose_diff_d(x_new, x_old, delta);
		
		{
			sm_debug("  icp_loop: killing. laser_sens has %d/%d rays valid,  %d corr found -> %d after double cut -> %d after adaptive cut \n", count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays, num_corr, num_corr2, num_corr_after);
			if(JJ) {
				jj_add_double_array("x_new", x_new, 3);
				jj_add_double_array("delta", delta, 3);
			}
		}
		/** Checks for oscillations */
		hashes[iteration] = ld_corr_hash(laser_sens);
		
		{
			sm_debug("  icp_loop: it. %d  hash=%d nvalid=%d mean error = %f, x_new= %s\n", 
				iteration, hashes[iteration], *valid, *total_error/ *valid, 
				friendly_pose(x_new));
		}

		
		/** PLICP terminates in a finite number of steps! */
		if(params->use_point_to_line_distance) {
			int loop_detected = 0; /* TODO: make function */
			int a; for(a=iteration-1;a>=0;a--) {
				if(hashes[a]==hashes[iteration]) {
					sm_debug("icpc: oscillation detected (cycle length = %d)\n", iteration-a);
					loop_detected = 1;
					break;
				}
			}
			if(loop_detected) {
				egsl_pop_named("icp_loop iteration");
				break;
			} 
		}
	
		/* This termination criterium is useless when using
		   the point-to-line-distance; however, we put it here because
		   one can choose to use the point-to-point distance. */
		if(_termination_criterion(params, delta)) {
			egsl_pop_named("icp_loop iteration");
			break;
		}
		
		copy_d(x_new, 3, x_old);
		copy_d(delta, 3, delta_old);
		
		
		egsl_pop_named("icp_loop iteration");
	}

	if(JJ) jj_loop_exit();
	
	*iterations = iteration+1;
	
	return all_is_okay;
}

int LaserScanMatcher::_ld_valid_fields(LDP ld)  {
	if(!ld) {
		sm_error("NULL pointer given as laser_data*.\n");	
		return 0;
	}
	
	int min_nrays = 10;
	int max_nrays = 10000;
	if(ld->nrays < min_nrays || ld->nrays > max_nrays) {
		sm_error("Invalid number of rays: %d\n", ld->nrays);
		return 0;
	}
	if(is_nan(ld->min_theta) || is_nan(ld->max_theta)) {
		sm_error("Invalid min / max theta: min_theta = %f max_theta = %f\n",
			ld->min_theta, ld->max_theta);
		return 0;
	}
	double min_fov = deg2rad(20.0); 
	double max_fov = 2.01 * M_PI;
	double fov = ld->max_theta - ld->min_theta;
	if( fov < min_fov || fov > max_fov) {
		sm_error("Strange FOV: %f rad = %f deg \n", fov, rad2deg(fov));
		return 0;
	}
	if(fabs(ld->min_theta - ld->theta[0]) > 1e-8) {
		sm_error("Min_theta (%f) should be theta[0] (%f)\n",
			ld->min_theta, ld->theta[0]);
		return 0;
	}
	if(fabs(ld->max_theta - ld->theta[ld->nrays-1]) > 1e-8) {
		sm_error("Min_theta (%f) should be theta[0] (%f)\n",
			ld->max_theta, ld->theta[ld->nrays-1]);
		return 0;
	}
	/* Check that there are valid rays */
	double min_reading = 0;
	double max_reading = 100;
	int i; for(i=0;i<ld->nrays;i++) {
		double th = ld->theta[i];
		if(ld->valid[i]) {
			double r = ld->readings[i];
			if(is_nan(r) || is_nan(th)) {
				sm_error("Ray #%d: r = %f  theta = %f but valid is %d\n",
					i, r, th, ld->valid[i]);
				return 0;
			}
			if( !( min_reading < r && r < max_reading ) ) {
				sm_error("Ray #%d: %f is not in interval (%f, %f) \n",
					i, r, min_reading, max_reading);
				return 0;
			}		
		} else {
			/* ray not valid, but checking theta anyway */
			if(is_nan(th)) {
				sm_error("Ray #%d: valid = %d  but theta = %f\n",
					i,  ld->valid[i], th);
				return 0;
			}

			if(ld->cluster[i] != -1 ) {
				sm_error("Invalid ray #%d has cluster %d\n.", i, ld->cluster[i]);
				return 0;
			}
		}
		if(ld->cluster[i] < -1 ) {
			sm_error("Ray #%d: Invalid cluster value %d\n.", i, ld->cluster[i]);
			return 0;
		}
		
		if(!is_nan(ld->readings_sigma[i]) && ld->readings_sigma[i] < 0) {
			sm_error("Ray #%d: has invalid readings_sigma %f \n", i, ld->readings_sigma[i]);
			return 0;
		}
		
	}
	/* Checks that there is at least 10% valid rays */
	int num_valid   = count_equal(ld->valid, ld->nrays, 1);
	int num_invalid = count_equal(ld->valid, ld->nrays, 0);
	if (num_valid < ld->nrays * min_perc_valid_rays_) {
		sm_error("Valid: %d/%d invalid: %d.\n", num_valid, ld->nrays, num_invalid);
		return 0;
	}

	return 1;
}

void LaserScanMatcher::_sm_icp(struct sm_params*params, struct sm_result*res) {
	res->valid = 0;

	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;
	
	if(!_ld_valid_fields(laser_ref) || 
	   !_ld_valid_fields(laser_sens)) {
		return;
	}
	
	sm_debug("sm_icp: laser_sens has %d/%d; laser_ref has %d/%d rays valid\n",
		count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
		count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays);
	
	
	/** Mark as invalid the rays outside of (min_reading, max_reading] */
	_ld_invalid_if_outside(laser_ref, params->min_reading, params->max_reading);
	_ld_invalid_if_outside(laser_sens, params->min_reading, params->max_reading);
	
	sm_debug("sm_icp:  laser_sens has %d/%d; laser_ref has %d/%d rays valid (after removing outside interval [%f, %f])\n",
		count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
		count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays,
   	   params->min_reading, params->max_reading);
	
	if(JJ) jj_context_enter("sm_icp");
	
	egsl_push_named("sm_icp");
	
			
	if(params->use_corr_tricks || params->debug_verify_tricks)
		ld_create_jump_tables(laser_ref);
		
	ld_compute_cartesian(laser_ref);
	ld_compute_cartesian(laser_sens);

	if(params->do_alpha_test) {
		ld_simple_clustering(laser_ref, params->clustering_threshold);
		ld_compute_orientation(laser_ref, params->orientation_neighbourhood, params->sigma);
		ld_simple_clustering(laser_sens, params->clustering_threshold);
		ld_compute_orientation(laser_sens, params->orientation_neighbourhood, params->sigma);
	}

	if(JJ) jj_add("laser_ref",  ld_to_json(laser_ref));
	if(JJ) jj_add("laser_sens", ld_to_json(laser_sens));
	
	gsl_vector * x_new = gsl_vector_alloc(3);
	gsl_vector * x_old = vector_from_array(3, params->first_guess);
	
	if(params->do_visibility_test) {
		sm_debug("laser_ref:\n");
		_visibilityTest(laser_ref, x_old);

		sm_debug("laser_sens:\n");
		gsl_vector * minus_x_old = gsl_vector_alloc(3);
		ominus(x_old,minus_x_old);
		_visibilityTest(laser_sens, minus_x_old);
		gsl_vector_free(minus_x_old);
	}
	
	double error;
	int iterations;
	int nvalid;
	if(!_icp_loop(params, x_old->data, x_new->data, &error, &nvalid, &iterations)) {
		// sm_error("icp: ICP failed for some reason. \n");
		ROS_ERROR("icp: ICP failed for some reason. \n");
		res->valid = 0;
		res->iterations = iterations;
		res->nvalid = 0;
		
	} else {
		/* It was succesfull */

		int restarted = 0;		
		double best_error = error;
		gsl_vector * best_x = gsl_vector_alloc(3);
		gsl_vector_memcpy(best_x, x_new);

		if(params->restart && 
			(error/nvalid)>(params->restart_threshold_mean_error) ) {
			sm_debug("Restarting: %f > %f \n",(error/nvalid),(params->restart_threshold_mean_error));
			restarted = 1;
			double dt  = params->restart_dt;
			double dth = params->restart_dtheta;
			sm_debug("icp_loop: dt = %f dtheta= %f deg\n",dt,rad2deg(dth));
		
			double perturb[6][3] = {
				{dt,0,0}, {-dt,0,0},
				{0,dt,0}, {0,-dt,0},
				{0,0,dth}, {0,0,-dth}
			};

			int a; for(a=0;a<6;a++){
				sm_debug("-- Restarting with perturbation #%d\n", a);
				struct sm_params my_params = *params;
				gsl_vector * start = gsl_vector_alloc(3);
					gvs(start, 0, gvg(x_new,0)+perturb[a][0]);
					gvs(start, 1, gvg(x_new,1)+perturb[a][1]);
					gvs(start, 2, gvg(x_new,2)+perturb[a][2]);
				gsl_vector * x_a = gsl_vector_alloc(3);
				double my_error; int my_valid; int my_iterations;
				if(!_icp_loop(&my_params, start->data, x_a->data, &my_error, &my_valid, &my_iterations)){
					sm_error("Error during restart #%d/%d. \n", a, 6);
					break;
				}
				iterations+=my_iterations;
		
				if(my_error < best_error) {
					sm_debug("--Perturbation #%d resulted in error %f < %f\n", a,my_error,best_error);
					gsl_vector_memcpy(best_x, x_a);
					best_error = my_error;
				}
				gsl_vector_free(x_a); gsl_vector_free(start);
			}
		}
	
	
		/* At last, we did it. */
		res->valid = 1;
		vector_to_array(best_x, res->x);
		sm_debug("icp: final x =  %s  \n", gsl_friendly_pose(best_x));
	
		if (restarted) { // recompute correspondences in case of restarts
			ld_compute_world_coords(laser_sens, res->x);
			if(params->use_corr_tricks)
				_find_correspondences_tricks(params);
			else
				_find_correspondences(params);
		}

		if(params->do_compute_covariance)  {

			val cov0_x, dx_dy1, dx_dy2;
			_compute_covariance_exact(
				laser_ref, laser_sens, best_x,
				&cov0_x, &dx_dy1, &dx_dy2);
		
			val cov_x = egsl_scale(square(params->sigma), cov0_x); 
			// egsl_v2da(cov_x, res->cov_x);
		
			res->cov_x_m = egsl_v2gslm(cov_x);
			res->dx_dy1_m = egsl_v2gslm(dx_dy1);
			res->dx_dy2_m = egsl_v2gslm(dx_dy2);
		
			if(0) {
				egsl_print("cov0_x", cov0_x);
				egsl_print_spectrum("cov0_x", cov0_x);
		
				val fim = ld_fisher0(laser_ref);
				val ifim = egsl_inverse(fim);
				egsl_print("fim", fim);
				egsl_print_spectrum("ifim", ifim);
			}
		}
	
		res->error = best_error;
		res->iterations = iterations;
		res->nvalid = nvalid;

		gsl_vector_free(best_x);
	}
	gsl_vector_free(x_new);
	gsl_vector_free(x_old);


	egsl_pop_named("sm_icp");

	if(JJ) jj_context_exit();
}

void LaserScanMatcher::_sm_icp_xy(struct sm_params*params, struct sm_result*res) 
{
	res->valid = 0;

	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;
	
	if(!ld_valid_fields(laser_ref) || 
	   !ld_valid_fields(laser_sens)) {
		return;
	}
	
    /*
	sm_debug("sm_icp: laser_sens has %d/%d; laser_ref has %d/%d rays valid\n",
		count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
		count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays);
	*/
	
	/** Mark as invalid the rays outside of (min_reading, max_reading] */
	_ld_invalid_if_outside(laser_ref, params->min_reading, params->max_reading);
	_ld_invalid_if_outside(laser_sens, params->min_reading, params->max_reading);
	
    /*
	sm_debug("sm_icp:  laser_sens has %d/%d; laser_ref has %d/%d rays valid (after removing outside interval [%f, %f])\n",
		count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
		count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays,
   	   params->min_reading, params->max_reading);
	
	if(JJ) jj_context_enter("sm_icp");
	
	egsl_push_named("sm_icp");
	*/
			
	if(params->use_corr_tricks || params->debug_verify_tricks)
		ld_create_jump_tables(laser_ref);
    /*		
	ld_compute_cartesian(laser_ref);
	ld_compute_cartesian(laser_sens);
    */

	if(params->do_alpha_test) {
		ld_simple_clustering(laser_ref, params->clustering_threshold);
		ld_compute_orientation(laser_ref, params->orientation_neighbourhood, params->sigma);
		ld_simple_clustering(laser_sens, params->clustering_threshold);
		ld_compute_orientation(laser_sens, params->orientation_neighbourhood, params->sigma);
	}

	if(JJ) jj_add("laser_ref",  ld_to_json(laser_ref));
	if(JJ) jj_add("laser_sens", ld_to_json(laser_sens));
	
	gsl_vector * x_new = gsl_vector_alloc(3);
	gsl_vector * x_old = vector_from_array(3, params->first_guess);
	
	if(params->do_visibility_test) {
		sm_debug("laser_ref:\n");
		_visibilityTest(laser_ref, x_old);

		sm_debug("laser_sens:\n");
		gsl_vector * minus_x_old = gsl_vector_alloc(3);
		ominus(x_old,minus_x_old);
		_visibilityTest(laser_sens, minus_x_old);
		gsl_vector_free(minus_x_old);
	}
	
	double error;
	int iterations;
	int nvalid;
	if(!_icp_loop(params, x_old->data, x_new->data, &error, &nvalid, &iterations)) {
		sm_error("icp: ICP failed for some reason. \n");
		res->valid = 0;
		res->iterations = iterations;
		res->nvalid = 0;
		
	} else {
		/* It was succesfull */
		
		double best_error = error;
		gsl_vector * best_x = gsl_vector_alloc(3);
		gsl_vector_memcpy(best_x, x_new);

		if(params->restart && 
			(error/nvalid)>(params->restart_threshold_mean_error) ) {
			sm_debug("Restarting: %f > %f \n",(error/nvalid),(params->restart_threshold_mean_error));
			double dt  = params->restart_dt;
			double dth = params->restart_dtheta;
			sm_debug("icp_loop: dt = %f dtheta= %f deg\n",dt,rad2deg(dth));
		
			double perturb[6][3] = {
				{dt,0,0}, {-dt,0,0},
				{0,dt,0}, {0,-dt,0},
				{0,0,dth}, {0,0,-dth}
			};

			int a; for(a=0;a<6;a++){
				sm_debug("-- Restarting with perturbation #%d\n", a);
				struct sm_params my_params = *params;
				gsl_vector * start = gsl_vector_alloc(3);
					gvs(start, 0, gvg(x_new,0)+perturb[a][0]);
					gvs(start, 1, gvg(x_new,1)+perturb[a][1]);
					gvs(start, 2, gvg(x_new,2)+perturb[a][2]);
				gsl_vector * x_a = gsl_vector_alloc(3);
				double my_error; int my_valid; int my_iterations;
				if(!_icp_loop(&my_params, start->data, x_a->data, &my_error, &my_valid, &my_iterations)){
					sm_error("Error during restart #%d/%d. \n", a, 6);
					break;
				}
				iterations+=my_iterations;
		
				if(my_error < best_error) {
					sm_debug("--Perturbation #%d resulted in error %f < %f\n", a,my_error,best_error);
					gsl_vector_memcpy(best_x, x_a);
					best_error = my_error;
				}
				gsl_vector_free(x_a); gsl_vector_free(start);
			}
		}
	
	
		/* At last, we did it. */
		res->valid = 1;
		vector_to_array(best_x, res->x);
		sm_debug("icp: final x =  %s  \n", gsl_friendly_pose(best_x));
	
	
		if(params->do_compute_covariance)  {

			val cov0_x, dx_dy1, dx_dy2;
			_compute_covariance_exact(
				laser_ref, laser_sens, best_x,
				&cov0_x, &dx_dy1, &dx_dy2);
		
			val cov_x = egsl_scale(square(params->sigma), cov0_x); 
			// egsl_v2da(cov_x, res->cov_x); 
		
			res->cov_x_m = egsl_v2gslm(cov_x);
			res->dx_dy1_m = egsl_v2gslm(dx_dy1);
			res->dx_dy2_m = egsl_v2gslm(dx_dy2);
		
			if(0) {
				egsl_print("cov0_x", cov0_x);
				egsl_print_spectrum("cov0_x", cov0_x);
		
				val fim = ld_fisher0(laser_ref);
				val ifim = egsl_inverse(fim);
				egsl_print("fim", fim);
				egsl_print_spectrum("ifim", ifim);
			}
		}
	
		res->error = best_error;
		res->iterations = iterations;
		res->nvalid = nvalid;

		gsl_vector_free(x_new);
		gsl_vector_free(x_old);
		gsl_vector_free(best_x);
	}
    /*
	egsl_pop_named("sm_icp");

	if(JJ) jj_context_exit();
    */
}

} // namespace scan_tools
