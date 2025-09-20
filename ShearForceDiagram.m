% Given data, need to change the respective lbfs and distances
distances = [4, 15, 22.5, 6.23, 12, 16.83, 8.53, 4];
lbfs = [-25, -100, -200, -1, -1, -500, -1, -25];
length=108; %all in IPS

% Compute RA and RB (reactions)
forces = sum(lbfs);
moments = sum(lbfs .* distances);
RB = -moments / length;
RA = -forces - RB;

% Cumulative positions of forces
distance_total = cumsum(distances);

% Total beam length
beam_length = 108;
dists = 1:beam_length;

% Initialize shear force diagram
shear_profile = zeros(1, beam_length);

% Start with RA as initial shear
current_shear = RA;
shear_profile(1:round(distance_total(1))) = current_shear;

% Loop over each load to compute shear drops
for i = 1:length(lbfs)
    % Get range for this section
    if i < length(lbfs)
        start_pos = round(distance_total(i)) + 1;
        end_pos = round(distance_total(i+1));
    else
        start_pos = round(distance_total(i)) + 1;
        end_pos = beam_length;
    end

    % Update shear value
    current_shear = current_shear + lbfs(i);

    % Assign shear value to this segment
    if start_pos <= beam_length
        shear_profile(start_pos:min(end_pos, beam_length)) = current_shear;
    end
end

% At the very end, apply RB to bring it back to 0 (for verification)
% You could also plot a final vertical drop or jump here if desired

% Plotting
figure;
plot(dists, shear_profile, 'r-', 'LineWidth', 2);
xlabel('Position along beam (inches)');
ylabel('Shear Force (lbf)');
title('Shear Force Diagram');
grid on;

