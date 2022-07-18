function [q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ] = noisy_Data()

[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion();

a = 0; b = 1;
rng(1)
r = a + (b-a).*rand(1,4);
e = .0001;
r = r/sum(r)*e;
r = r + [0 e/2 e/4 e/4];
q_Robot_config = q_Robot_config - r;

rng(2)
r = a + (b-a).*rand(1,4);
r = r/sum(r)*e;
r = r + [0 e/2 e/4 e/4];
q_camera_config = q_camera_config - r;

rng(3)
r = a + (b-a).*rand(10,3);
r = r*0.0003;
t_Robot_config = t_Robot_config - r;

rng(4)
r = a + (b-a).*rand(10,3);
r = r*0.0005;
t_camera_config = t_camera_config - r;
end