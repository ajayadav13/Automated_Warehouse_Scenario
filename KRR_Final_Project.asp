%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  GENERAL DEFINITION  %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%Object Declaration and sorting%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% NODE %%%
state(NodeId,pair(X,Y)):- init(object(node,NodeId),value(at,pair(X,Y))).
node(NodeId):- init(object(node,NodeId),value(at,pair(X,Y))).
node_Num(N):- N=#count{NodeId:init(object(node,NodeId),value(at,pair(X,Y)))}.


%%% GRobotId %%%
state(object(highway,NodeId)):- init(object(highway,NodeId),value(at,pair(X,Y))).
num_Col(num_cols):- num_cols = #count{X:init(object(node,NodeId),value(at,pair(X,Y)))}.
num_Row(num_rows):- num_rows = #count{Y:init(object(node,NodeId),value(at,pair(X,Y)))}.


%%% ROBOT %%%
state(object(robot,RobotId),on(node,NodeId),0):- state(NodeId,pair(X,Y)), init(object(robot,RobotId),value(at,pair(X,Y))).

robot(RobotId):- init(object(robot,RobotId),value(at,pair(X,Y))).

move(1,0;-1,0;0,1;0,-1).

numRobots(Num):- Num = #count{RobotId:init(object(robot,RobotId),value(at,pair(X,Y)))}.

{occurs(object(robot,RobotId),move(X1,Y1),T):move(X1,Y1)}1:- numRobots(Num), RobotId=1..Num, T=0..n-1.

state(object(robot,RobotId),on(node, Moved_NodeId), T+1):- state(NodeId,pair(X,Y)), 
state(object(robot,RobotId),on(node,NodeId),T), occurs(object(robot,RobotId),move(X1,Y1),T), state(Moved_NodeId, pair(X+X1,Y+Y1)).


%%% ORDER %%
state(OrdeRobotId,object(node,NodeId),product_info(ProductId,PQ),0):- init(object(order,OrdeRobotId),value(pickingStation,PKid)), state(object(pickingStation,PKid),object(node,NodeId)), init(object(order,OrdeRobotId),value(line,pair(ProductId,PQ))).


%%% PICKING STATION %%%
state(object(pickingStation,PKid),object(node,NodeId)) :- init(object(pickingStation,PKid),value(at,pair(X,Y))), init(object(node,NodeId),value(at,pair(X,Y))).


%%% SHELF %%%
shelf(ShelfId):- init(object(shelf,ShelfId),value(at,pair(X,Y))).
state(object(shelf,ShelfId),on(node,NodeId),0) :- init(object(shelf,ShelfId),value(at,pair(X,Y))), state(NodeId,pair(X,Y)).


%%% PRODUCT %%%
product(ProductId):- init(object(product,ProductId),value(on,pair(ShelfId,EA))).
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,EA),0):- init(object(product,ProductId),value(on,pair(ShelfId,EA))).
numProducts(Num):- Num=#count{ProductId:init(object(product,ProductId),value(on,pair(X,Y)))}.



%%% PICKUP %%%
% Need to specifiy the shelf
{occurs(pickup(RobotId,ShelfId),T):shelf(ShelfId)}1:- RobotId=1..NR, numRobots(NR), T=0..n-1.
occurs(object(robot,RobotId),pickup,T):- occurs(pickup(RobotId,_),T).
% Effect of picking up a shelf
state(object(shelf,ShelfId),on(robot,RobotId),T+1) :- occurs(object(robot,RobotId),pickup,T), state(object(shelf,ShelfId),on(node,NodeId),T), state(object(robot,RobotId),on(node,NodeId),T), R=1..NR, numRobots(NR).


%%% PUTDOWN %%%
{occurs(putdown(RobotId,ShelfId),T):shelf(ShelfId)}1:- RobotId=1..NR, numRobots(NR), T=0..TN,TN=n-1.
occurs(object(robot,RobotId),putdown,T):-occurs(putdown(RobotId,_),T).
% Effect of putting down a shelf
state(object(shelf,ShelfId),on(node,NodeId),T+1) :- occurs(putdown(RobotId,ShelfId),T), state(object(shelf,ShelfId),on(robot,RobotId),T), state(object(robot,RobotId),on(node,NodeId),T).

%%%%% Shelves follow the COMMONSENSE LAW OF INERTIA since the position of shelves is dependent on the position of the robots%%%%%%%%%%%%%%%%%%

locations(1..3). robots(1..2). shelfs(1..2).

% Define a rule stating that a shelf is at the same location as a robot
shelf_at_Pos(R, L) :- robots(R), locations(L).

% Define a rule stating that a robot is at a location
robot_at(R, L) :- robots(R), locations(L).

% A shelf is at the same location as a robot
:- not shelf_at_Pos(R, L), robot_at(R, L).




%%% DELIVERY %%%
{occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,EA),T):state(OrdeRobotId,object(node,NodeId),product_info(ProductId,EA),T), 
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T), EA=1..PQ}1:- RobotId=1..NR, numRobots(NR), T=0..TN,TN=n-1.
occurs(object(robot,RobotId),deliver(OrdeRobotId,ProductId,EA),T):-occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,EA),T).
state(OrdeRobotId,object(node,NodeId),product_info(ProductId,EA-QQ),T+1):- occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,QQ),T), 
state(OrdeRobotId,object(node,NodeId),product_info(ProductId,EA),T).
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ-QQ),T+1):- occurs(object(robot,R),for(order,OI),deliver(ShelfId,ProductId,QQ),T), 
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T).


%%%%%%%%%%%%%%%%%%%%%  GOAL  %%%%%%%%%%%%%%%%%%%%%
:- not state(OrdeRobotId,object(node,NodeId),product_info(ProductId,0),T), state(OrdeRobotId,object(node,NodeId),product_info(ProductId,_),0), 
NodeId=1..NN, node_Num(NN), T=n.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  COMMONSENSE LAW OF INERTIA  %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state(object(robot,RobotId),on(node,NodeId),T+1):- state(object(robot,RobotId),on(node,NodeId),T), not occurs(object(robot,RobotId),move(_,_),T), T<n.
state(OrdeRobotId,object(node,NodeId),product_info(ProductId,OQ),T+1):- state(OrdeRobotId,object(node,NodeId),product_info(ProductId,OQ),T), 
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T), not occurs(object(robot,_),for(order,OrdeRobotId),deliver(ShelfId,ProductId,_),T), T<n.
state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T+1):- state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T), 
not occurs(object(robot,_),for(order,_),deliver(ShelfId,ProductId,_),T), T<n.
state(object(shelf,ShelfId),on(robot,RobotId),T+1):-state(object(shelf,ShelfId),on(robot,RobotId),T), not occurs(putdown(RobotId,ShelfId),T), T<n.
state(object(shelf,ShelfId),on(node,NodeId),T+1):-state(object(shelf,ShelfId),on(node,NodeId),T), not occurs(pickup(_,ShelfId),T), T<n.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  CONSTRAINTS  %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%  ROBOT  %%%%%%%%%%%%%%%%%%%%%
% Cannot move to more than 1 slot apart
:- not move(1,0;-1,0;0,1;0,-1).
:- occurs(object(robot,R),move(DX,DY),T), DX>1, DX<-1, DY>1, DY<-1.
% Within the range of gRobotId
:- occurs(object(robot,RobotId),move(X1,Y1),T), state(object(robot,RobotId),on(node,NodeId),T), state(NodeId,pair(X,Y)), X+X1>NC, num_Col(NC).
:- occurs(object(robot,RobotId),move(X1,Y1),T), state(object(robot,RobotId),on(node,NodeId),T), state(NodeId,pair(X,Y)), Y+Y1>NR, num_Row(NR).
:- occurs(object(robot,RobotId),move(X1,Y1),T), state(object(robot,RobotId),on(node,NodeId),T), state(NodeId,pair(X,Y)), X+X1<1.
:- occurs(object(robot,RobotId),move(X1,Y1),T), state(object(robot,RobotId),on(node,NodeId),T), state(NodeId,pair(X,Y)), Y+Y1<1.
% Cannot do pickup/putdown/deliver and move concurrently
:- occurs(object(robot,RobotId),move(X1,Y1),T), occurs(pickup(RobotId,_),T).
:- occurs(object(robot,RobotId),move(X1,Y1),T), occurs(putdown(RobotId,_),T).
:- occurs(object(robot,RobotId),move(X1,Y1),T), occurs(object(robot,RobotId),for(order,_),deliver(_,_,_),T).
% Cannot move under other shelves if a robot holds a shelf already
:- state(object(robot,RobotId),on(node,NodeId),T), occurs(object(robot,RobotId),move(DX,DY),T), state(NodeId,pair(X,Y)), state(object(shelf,ShelfId),on(robot,RobotId),T), 
state(object(shelf,ShelfId_other),on(node,NodeId_other),T), state(NodeId_other,pair(X+DX,Y+DY)), ShelfId!=ShelfId_other, NodeId!=NodeId_other.
% No two actions concurrently
:- occurs(object(robot,RobotId),Move,T), occurs(object(robot,RobotId),Move_other,T), Move!=Move_other.


%%%%%%%%%%%%%%%%%%%%%  PICKUP  %%%%%%%%%%%%%%%%%%%%%
% No 2 shelves on the same location
:- state(object(shelf,ShelfId),on(node,NodeId),T), state(object(shelf,ShelfId2),on(node,NodeId),T), ShelfId!=ShelfId2.
:- state(object(shelf,ShelfId),on(robot,RobotId),T), state(object(shelf,ShelfId2),on(robot,RobotId),T), ShelfId!=ShelfId2.
% Cannot pick up the shelve more than 1
:- occurs(pickup(RobotId,ShelfId),T), state(object(shelf,ShelfId_other),on(robot,RobotId),T), ShelfId!=ShelfId_other.
% Cannot take away the shelf from other robot
:- occurs(pickup(RobotId,ShelfId),T), state(object(shelf,ShelfId),on(robot,RobotId_other),T), RobotId!=RobotId_other.
% No 2 robots pick up the same shelf
:- 2{occurs(pickup(RobotId,ShelfId),T): robot(RobotId)}, shelf(ShelfId).
% Can pick a shelf only on the node where the shelf reShelfIde
:- occurs(pickup(RobotId,ShelfId),T), state(object(shelf,ShelfId),on(node,NodeId),T), not state(object(robot,RobotId),on(node,NodeId),T). 


%%%%%%%%%%%%%%%%%%%%%  PUTDOWN  %%%%%%%%%%%%%%%%%%%%%
% Robot can putdown only if they holds a shelf.
:- occurs(putdown(RobotId,ShelfId),T), not state(object(shelf,ShelfId),on(robot,RobotId),T).
% No putdown a shelf on the highway
:- occurs(putdown(RobotId,ShelfId),T), state(object(robot,RobotId),on(node,NodeId),T), state(object(highway,NodeId)). 


%%%%%%%%%%%%%%%%%%%%%  DELIVERY  %%%%%%%%%%%%%%%%%%%%%
% Robot can do deliver only when it holds a shelves containing product
:- occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,_),T), state(object(product,ProductId),object(shelf,ShelfId),with(quantity,_),T), not state(object(shelf,ShelfId),on(robot,RobotId),T).
% Robot can do deliver only when it is at corresponding pickingstation
:- occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(_,ProductId,_),T), state(OrdeRobotId,object(node,NodeId),product_info(ProductId,_),T), not state(object(robot,RobotId),on(node, NodeId),T), 
state(object(pickingStation,PKid),object(node,NodeId)).
% Robot cannot do deliver more than product amount
:- occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,QQ),T), state(object(product,ProductId),object(shelf,ShelfId),with(quantity,PQ),T), QQ>PQ.
% Robot cannot do deliver if order amount is larger than product amount
:- occurs(object(robot,RobotId),for(order,OrdeRobotId),deliver(ShelfId,ProductId,QQ),T), state(OrdeRobotId,object(node,NodeId),product_info(ProductId,PQ),T), QQ>PQ.


%%%%%%%%%%%%%%%%%%    ROBOT STATES     %%%%%%%%%%%%%%%%%%
% Cannot swap the location = no collision
:- state(object(robot,RobotId1),on(node,NodeId),T), state(object(robot,RobotId1),on(node,NodeId2),T+1), state(object(robot,RobotId2),on(node,NodeId2),T), state(object(robot,RobotId2),on(node,NodeId),T+1), RobotId1!=RobotId2.
% No 2 robots are on same location
:- 2{state(object(robot,RobotId),on(node,NodeId),T):node(NodeId)}, robot(RobotId), T=0..n.
:- 2{state(object(robot,RobotId),on(node,NodeId),T):robot(RobotId)}, node(NodeId), T=0..n.


%%%%%%%%%%%%%%%%%%    SHELF STATES     %%%%%%%%%%%%%%%%%%
% Shlef cannot be at both robot and node concurrently
:- state(object(shelf,ShelfId),on(node,_),T), state(object(shelf,ShelfId),on(robot,_),T).
:- state(object(shelf,ShelfId),on(node,NodeId),T), state(object(shelf,ShelfId),on(node,NodeId2),T), NodeId!=NodeId2.
:- state(object(shelf,ShelfId),on(robot,RobotId),T), state(object(shelf,ShelfId),on(robot,RobotId2),T), RobotId!=RobotId2.
timestamp(N):-N=#count{T:occurs(A,B,T)}.
#minimize{N: timestamp(N)}.
#show occurs/3.
