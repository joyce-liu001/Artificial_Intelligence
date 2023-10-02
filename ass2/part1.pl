% z5271687
% Zhaoyan LIU

% Question 1.1: List Processing 
% check number is even or odd
even(Num) :- 0 is mod(Num, 2).
odd(Num) :- 1 is mod(Num, 2).

% Base case
% The sum for an empty list should be 0.
sumsq_even([], 0).

% When the first element is even, sums the squares of head
sumsq_even([Head|Tail], Sum) :-
	even(Head),
    sumsq_even(Tail, Tempsum),
    Sum is Tempsum + (Head*Head).
   
% Otherwise the first element is odd
% go to next element
sumsq_even([Head|Tail], Sum) :-
	odd(Head),
    sumsq_even(Tail, Sum).

% Question 1.2: Planning 
% State of the robot's world = state(RLoc, RHC, SWC, MW, RHM)
% action(Action, State, NewState): Action in State produces NewState
% RLoc - Robot Location, RHC - Robot Has Coffee, SWC - Sam Wants Coffee, MW - Mail Waiting, RHM - Robot Has Mail

position_mc([lab, mr, cs, off]).        % list positions in mc order
position_mcc([off, cs, mr, lab]).       % list positions in mcc order  

move_next(List, Pos, Next, mc) :-
    last(List, Pos),                    % when the position is last 
    position_mc([Next|_]).              % go the first element

move_next(List, Pos, Next, mcc) :-
    last(List, Pos),                    % when the position is last  
    position_mcc([Next|_]).             % when the position is last

move_next([Pos , Next | _], Pos, Next, _). % first element is the position

move_next([_ | Others], Pos, Next, MC):-   % literate until find
    move_next(Others, Pos, Next, MC).

action(pum,						        % Pick up mail
	state(mr, RHC, SWC, true, false),	% Before action, robot at mail room and mail is waiting but robot hasn't mail
	state(mr, RHC, SWC, false, true)).	% After action, robot got the mail

action(dm,						        % Deliver mail 
	state(off, RHC, SWC, MW, true),	    % Before action, robot has mail at office
    state(off, RHC, SWC, MW, false)).   % After action, robot does not have mail

action(puc,				                % Pick up Coffee
	state(cs, false, SWC, MW, RHM),		% Before action, robot at coffee shop but robot not have coffee
    state(cs, true, SWC, MW, RHM)).     % After action, robot has coffee

action(dc,				                % Deliver coffee
	state(off, true, _, MW, RHM),		% Before action, robot has coffee at office
	state(off, false, false, MW, RHM)).	% After action, robot does not have coffee and Sam does not want Coffee

% find the next postion and update 
action(mc,                              % move clockwise
   	state(Pos, RHC, SWC, MW, RHM),
    state(NextPos, RHC, SWC, MW, RHM)) :-
        position_mc(PosList),           % get the list of positions
        move_next(PosList, Pos, NextPos, mc).
    
action(mcc,                             % move counterclockwise
    state(Pos, RHC, SWC, MW, RHM),
    state(NextPos, RHC, SWC, MW, RHM)) :-
        position_mcc(PosList),          % get the list of positions 
        move_next(PosList, Pos, NextPos, mcc).


% basecase
plan(State, State, []).				% To achieve State from State itself, do nothing

plan(State1, GoalState, [Action1 | RestofPlan]) :-
	action(Action1, State1, State2),	    % Make first action resulting in State2
	plan(State2, GoalState, RestofPlan). 	% Find rest of plan

% Iterative deepening planner
% Backtracking to "append" generates lists of increasing length
% Forces "plan" to ceate fixed length plans

id_plan(Start, Goal, Plan) :-
    append(Plan, _, _),
    plan(Start, Goal, Plan).


% Question 1.3(a): 
intra_construction(C1 <- B1, C2 <- B2, C1 <- Z1B, C <- B3, C <- B4) :-
    C1 = C2,		            % C1 is equal to C2
    intersection(B1, B2, B),    % find the interaction
    gensym(z, C),
    subtract(B1, B, B3),	    % distributes the differences to two new clauses.
    subtract(B2, B, B4),
    append(B, [C], Z1B). 

% Question 1.3 (b):
absorption(C1 <- B1, C2 <- B2, C1 <- B3, C2 <- B2) :-
    C1 \= C2,
    intersection(B1, B2, Comm),
    % the second clause is unchanged, so the second is always smaller one
    % the body of smaller clause is a subset of the larger
    Comm = B2,
    subtract(B1, Comm, Diff), % common elements can be removed from the larger clause
    append([C2], Diff, B3). % replaced by the head of the smaller one

% Question 1.3 (c):
truncation(C1 <- B1, C2 <- B2, C1 <- Comm) :-
    C1 = C2,
    % the body of the new clause is just the intersection of the bodies of the input clauses.
    intersection(B1, B2, Comm). 