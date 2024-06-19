function targets = RRTstar(InputGoal,robot,obstacle)

% Velikost prostoru
xmax = 2*pi;
ymax = 2*pi;
zmax = 2*pi;


%% Z�kladn� parametry

EPS = 1.5*pi;                 % Vzd�lenost epsilon, kter� omezuje d�lku p��mky mezi n�hodn�m bodem a rodi�ovsk�m bodem
NodesMax = 25;             % Maxim�ln� po�et bod�
r=2*pi;                     % Polom�r r pro RRT*, kter� ur�uje vzd�lenost, ve kter� dojde ke zkr�cen� cesty 

% definov�n� Startu a C�le
start.position = [0 , 0, 0];
start.value = 0;
start.parent = 0;
goal.position = InputGoal
goal.value = 0;

nodes(1) = start;
figure(1);
grid on
axis([-2*pi xmax -2*pi ymax -2*pi zmax]);
newNode.value=0;
hold on


%% RRT ��st
    
for i = 1:1:NodesMax                                                                    % Hlavn� cyklus, ve kter�m prob�h� vytv��en� nov�ho bodu a jeho p�ipojen� ke stromu.
  
    randomNode = [rand(1)*4*pi - 2*pi rand(1)*4*pi - 2*pi  rand(1)*4*pi - 2*pi];         % Vytvo�en� n�hodn�ho bodu
    plot3(randomNode(1),randomNode(2),randomNode(3), '*','Color', [0 0.5 1])           % Mo�n� vykreslen� n�hodn�ho bodu

    distance = [];                                  % Vytvo�en� pr�zdn�ho vektoru distance
  
    for j=1:1:length(nodes)                         % Cyklus for, kter� z�sk� vzd�lenost ka�d�ho bodu od n�hodn�ho
        n = nodes(j);
        tmp = dist(n.position, randomNode);
        distance = [distance tmp];                  % Vzd�lenosti zapisujeme do vektoru 
    end     
   
   [val, idx] = min(distance);                      % Vyjmut� minim�ln� vzd�lenosti a p�edev��m jej� pozice ve vektoru (Zp�tn� lze dohledat nejbli��� bod)
    nearNode = nodes(idx);                          % Tento bod nazveme nearNode
       
    newNode.position = steer(randomNode, nearNode.position, val, EPS);                  % Vytvo��me newNode na spojnici randomNode a nearNode ve vzd�lenosti EPS od nearNode
 
    if noCollision(newNode.position,nearNode.position,robot,obstacle)                                   % Ov���me, jestli mezi nearNode a randomNode 
                                                                                        
        line ([nearNode.position(1), newNode.position(1)], [nearNode.position(2), newNode.position(2)],[nearNode.position(3), newNode.position(3)], 'Color', 'k', 'LineWidth', 1);          % Vykreslov�n�
        drawnow
        hold on
        newNode.value = dist(newNode.position, nearNode.position) + nearNode.value;               % Nov�mu bodu je p�i�azena hodnota, co� je jeho vzd�lenost od nearNode + hodnota samotn�ho bodu nearNode

%%      RRT* ��st

        nearestNode = [];                          % Vytvo�en� vektoru nearestNode                                    
        neighborNodes = 1;                         % Vytvo�en� prom�nn� neighborNodes
        
        for j = 1:1:length(nodes)                  % V tomto cyklu se vytypujou v�echny body, kter� by nevedly ke kolizi a nach�z� se v polom�ru r

            if  noCollision(nodes(j).position,newNode.position,robot,obstacle) && dist(nodes(j).position, newNode.position) <= r
                nearestNode(neighborNodes).position = nodes(j).position;            % poka�d�, kdy� je n�kter� z t�chto bod� nalezen je zaps�n do vektoru
                nearestNode(neighborNodes).value = nodes(j).value;                  % a je mu p�i�azena i hodnota             
                neighborNodes = neighborNodes+1;                                    % Nav��� se po�et neighborNodes
             end

        end
        
        minNode = nearNode;                 % Nejbli��� bod je prozat�m p�vodn� nearNode v p��pad�, �e ��dn� jin� nesplnil podm�nku
        minCost = newNode.value;            % Minim�ln� hodnota je prozatimn� hodnota 
        
        for k = 1:1:length(nearestNode)     % V tomto cyklu se porovnaj� v�echny vzd�lenosti newNode a nearestNode. V p��pad� �e je vzd�lenost ni��� ne� aktu�ln�, tak je minCost a minNode p�eps�na

           if   nearestNode(k).value + dist(nearestNode(k).position, newNode.position) < minCost
                minNode = nearestNode(k);
                minCost = nearestNode(k).value + dist(nearestNode(k).position, newNode.position);
                hold on
            end
        end
        
        
        for j = 1:1:length(nodes)           % Tento cyklus zjist�, kter� node(j) m� stejnou pozici jako minNode (u kter� v�me, �e je nejbli��� soused)
            if nodes(j).position == minNode.position
                newNode.parent = j;         % Tento bod se stane nov�m parent bodem
            end
        end
        
        if  noCollision(newNode.position,goal.position,robot,obstacle) 
            nodes =  [nodes newNode];
            break
        else
        nodes =  [nodes newNode];           % Nov� vytvo�en� bod je p�ips�n do stromu a cyklus pokra�uje vytvo�en�m dal��ho n�hodn�ho bodu
        end
        % nodes =  [nodes newNode];

    end

 end

D = [];                           % Vytvo�en� vektoru D
for j = 1:1:length(nodes)         % V tomto cyklu se m��� vzd�lenost v�ech bod� ve strom� k c�li
    if noCollision(nodes(j).position,goal.position,robot,obstacle)
    tempDistance = dist(nodes(j).position, goal.position);
    else
    tempDistance = NaN
    end
    D = [D tempDistance]         % Op�t zaps�no do p�ipraven�ho vektoru D
end

[val, idx] = min(D);              % Op�t pomoc� indexu a funkce min je zji�t�na pozice nejbli���ho bodu
bestpath = nodes(idx);                      
goal.parent = idx;                % Tento bod se st�v� rodi�ovsk�m bodem c�le
End = goal;                       
nodes = [nodes goal];             % C�l je p�i�azen do stromu
origin = start.position
targets = [];

while End.parent ~= 0             % Dokud nen� rodi� Endu 0, cyklus zp�tn� mapuje nejkrat�� cestu od c�le k po��tku a vyresluje trasu
    start = End.parent;           % Prom�nn� start si zap�e index parent bodu
    line([End.position(1), nodes(start).position(1)], [End.position(2), nodes(start).position(2)],[End.position(3), nodes(start).position(3)], 'Color', 'r', 'LineWidth', 2.5);
    hold on
    targets = [targets; End.position(1), End.position(2),End.position(3)];
    End = nodes(start);           % Zde je index pou�it, aby pro dal�� cyklus byl End pr�v� tento parent bod         
end                               % Tento cyklus trv� ne� je parent 0, neboli startovn� bod

targets = [targets; origin]


%% Funkce

function d = dist(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2 + (q1(3)-q2(3))^2);
end

function A = steer(qr, qn, val, eps)
   qnew = [0 0 0];
   
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
       qnew(3) = qn(3) + ((qr(3)-qn(3))*eps)/dist(qr,qn);
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
       qnew(3) = qr(3);
   end   
   A = [qnew(1), qnew(2), qnew(3)];
end


%% Kolizn� funkce 


function collision = noCollision(point1,point2,robot,obstacle)

 x = linspace(point1(1),point2(1),20);
 y = linspace(point1(2),point2(2),20);
 z = linspace(point1(3),point2(3),20);
 collision = 1;

 for  i = 1:1:20
    
    config = [x(i) y(i) z(i)];

    if checkCollisionState(robot,config,obstacle);
    collision = 0;
    else
    collision = 1;
    end

 end
end

end


