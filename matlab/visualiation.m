clear;
figure;
hold all;
[x,y,z] = sphere();
r = 138;
h = surf(r*x, r*y, r*z);
set(h, 'FaceAlpha', 0.5);
colormap(bone);
plot3([-134.187 , -117.338], [22.9009, 66.2314], [-22.6495, -29.8161], 'LineWidth', 4);
plot3([-39.7713 , 7.46237], [102.933, 101.775], [-82.8674, -92.8978], 'LineWidth', 4);
plot3([18.6745 , 64.9984], [17.4914, 7.70827], [-135.606, -121.487], 'LineWidth', 4);
plot3([47.3574 , 2.27829], [-80.0088, -97.346], [-101.975, -97.7855], 'LineWidth', 4);
plot3([-39.9231 , -60.049], [-34.4075, 10.6659], [-127.538, -123.79], 'LineWidth', 4);
