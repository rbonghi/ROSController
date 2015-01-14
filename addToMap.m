function addToMap( pose,vel )
%ADDINFOMAP Summary of this function goes here
%   Detailed explanation goes here

plotICR(pose,vel);

end

function plotICR(poseData,velData)
p = poseData(end,:);
vm = velData(end,:);
%vm = mean(h.velData);
center = [p(1) p(2)];
if vm(2) ~= 0
    r = vm(1)/vm(2);
    center = [p(1)-r/2*sin(p(3)) p(2)+r/2*cos(p(3))];
end
plot(center(1),center(2),'o','Color','red');
text(center(1),center(2),'\leftarrow ICR');
%global Explorer;
%Explorer.setCenter(center);
end