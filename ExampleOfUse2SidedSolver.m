function [OptimalPath] = ExampleOfUse2SidedSolver()


I=imread('occupancygrid.bmp');
I=flipud(I)
%imshow(I);axis on
set(gca,'YDir','normal')


image = I
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
grid = robotics.BinaryOccupancyGrid(bwimage,30);
robotRadius = 0.37;
mapInflated = copy(grid);
inflate(mapInflated,robotRadius);
matriz = occupancyMatrix(mapInflated);




MAP=matriz;




%Start Positions
StartX=50;
StartY=100;

% a=ASl
%Start Positions
% StartX=5;
% StartY=99;


GoalX=580;
GoalY=50;


Connecting_Distance=11;


load NeighboorsTable2 NeighboorsTable
Neighboors=NeighboorsTable{Connecting_Distance};


OptimalPath=ASTARPATH2SIDED(StartX,StartY,MAP,GoalX,GoalY,Connecting_Distance,Neighboors)
if size(OptimalPath,2)>1
figure(2)
imagesc(MAP)

    colormap(flipud(gray));

hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Goal','Start','Path')


else 
     pause(1);
 h=msgbox('Path nao existe!','warn');
 uiwait(h,5);
 end









showNeighboors=1; %Setar pra 1 para possiveis path
if showNeighboors==1
        figure('name','Con1')
PlotConnectors(Connecting_Distance)

OptimalPath

end
end