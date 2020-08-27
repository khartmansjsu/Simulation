Domain_height = 10;
Domain_length = 48;

spatial_resolution = 1/20;

tic
for i = Domain_height/spatial_resolution:-1:1
    for j = Domain_length/spatial_resolution:-1:1
        domain(i,j).type = ''; domain(i,j).V = []; domain(i,j).phi = 1;
    end
end
toc


%% wall boundary conditions

if wall_type == 'noslip'
    
    
end

if wall_type == 'freeslip'
    
    
    
end




