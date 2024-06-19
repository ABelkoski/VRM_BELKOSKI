function targets = RRTstar(InputGoal,robot,obstacle)

% Velikost prostoru
xmax = 2*pi;
ymax = 2*pi;
zmax = 2*pi;


%% Základní parametry

EPS = 1.5*pi;                 % Vzdálenost epsilon, která omezuje délku pøímky mezi náhodným bodem a rodièovským bodem
NodesMax = 25;             % Maximální poèet bodù
r=2*pi;                     % Polomìr r pro RRT*, který urèuje vzdálenost, ve které dojde ke zkrácení cesty 

% definování Startu a Cíle
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


%% RRT èást
    
for i = 1:1:NodesMax                                                                    % Hlavní cyklus, ve kterém probíhá vytváøení nového bodu a jeho pøipojení ke stromu.
  
    randomNode = [rand(1)*4*pi - 2*pi rand(1)*4*pi - 2*pi  rand(1)*4*pi - 2*pi];         % Vytvoøení náhodného bodu
    plot3(randomNode(1),randomNode(2),randomNode(3), '*','Color', [0 0.5 1])           % Možné vykreslení náhodného bodu

    distance = [];                                  % Vytvoøení prázdného vektoru distance
  
    for j=1:1:length(nodes)                         % Cyklus for, který získá vzdálenost každého bodu od náhodného
        n = nodes(j);
        tmp = dist(n.position, randomNode);
        distance = [distance tmp];                  % Vzdálenosti zapisujeme do vektoru 
    end     
   
   [val, idx] = min(distance);                      % Vyjmutí minimální vzdálenosti a pøedevším její pozice ve vektoru (Zpìtnì lze dohledat nejbližší bod)
    nearNode = nodes(idx);                          % Tento bod nazveme nearNode
       
    newNode.position = steer(randomNode, nearNode.position, val, EPS);                  % Vytvoøíme newNode na spojnici randomNode a nearNode ve vzdálenosti EPS od nearNode
 
    if noCollision(newNode.position,nearNode.position,robot,obstacle)                                   % Ovìøíme, jestli mezi nearNode a randomNode 
                                                                                        
        line ([nearNode.position(1), newNode.position(1)], [nearNode.position(2), newNode.position(2)],[nearNode.position(3), newNode.position(3)], 'Color', 'k', 'LineWidth', 1);          % Vykreslování
        drawnow
        hold on
        newNode.value = dist(newNode.position, nearNode.position) + nearNode.value;               % Novému bodu je pøiøazena hodnota, což je jeho vzdálenost od nearNode + hodnota samotného bodu nearNode

%%      RRT* èást

        nearestNode = [];                          % Vytvoøení vektoru nearestNode                                    
        neighborNodes = 1;                         % Vytvoøení promìnné neighborNodes
        
        for j = 1:1:length(nodes)                  % V tomto cyklu se vytypujou všechny body, které by nevedly ke kolizi a nachází se v polomìru r

            if  noCollision(nodes(j).position,newNode.position,robot,obstacle) && dist(nodes(j).position, newNode.position) <= r
                nearestNode(neighborNodes).position = nodes(j).position;            % pokaždé, když je nìkterý z tìchto bodù nalezen je zapsán do vektoru
                nearestNode(neighborNodes).value = nodes(j).value;                  % a je mu pøiøazena i hodnota             
                neighborNodes = neighborNodes+1;                                    % Navýší se poèet neighborNodes
             end

        end
        
        minNode = nearNode;                 % Nejbližší bod je prozatím pùvodní nearNode v pøípadì, že žádný jiný nesplnil podmínku
        minCost = newNode.value;            % Minimální hodnota je prozatimní hodnota 
        
        for k = 1:1:length(nearestNode)     % V tomto cyklu se porovnají všechny vzdálenosti newNode a nearestNode. V pøípadì že je vzdálenost nižší než aktuální, tak je minCost a minNode pøepsána

           if   nearestNode(k).value + dist(nearestNode(k).position, newNode.position) < minCost
                minNode = nearestNode(k);
                minCost = nearestNode(k).value + dist(nearestNode(k).position, newNode.position);
                hold on
            end
        end
        
        
        for j = 1:1:length(nodes)           % Tento cyklus zjistí, která node(j) má stejnou pozici jako minNode (u které víme, že je nejbližší soused)
            if nodes(j).position == minNode.position
                newNode.parent = j;         % Tento bod se stane novým parent bodem
            end
        end
        
        if  noCollision(newNode.position,goal.position,robot,obstacle) 
            nodes =  [nodes newNode];
            break
        else
        nodes =  [nodes newNode];           % Novì vytvoøený bod je pøipsán do stromu a cyklus pokraèuje vytvoøením dalšího náhodného bodu
        end
        % nodes =  [nodes newNode];

    end

 end

D = [];                           % Vytvoøení vektoru D
for j = 1:1:length(nodes)         % V tomto cyklu se mìøí vzdálenost všech bodù ve stromì k cíli
    if noCollision(nodes(j).position,goal.position,robot,obstacle)
    tempDistance = dist(nodes(j).position, goal.position);
    else
    tempDistance = NaN
    end
    D = [D tempDistance]         % Opìt zapsáno do pøipraveného vektoru D
end

[val, idx] = min(D);              % Opìt pomocí indexu a funkce min je zjištìna pozice nejbližšího bodu
bestpath = nodes(idx);                      
goal.parent = idx;                % Tento bod se stává rodièovským bodem cíle
End = goal;                       
nodes = [nodes goal];             % Cíl je pøiøazen do stromu
origin = start.position
targets = [];

while End.parent ~= 0             % Dokud není rodiè Endu 0, cyklus zpìtnì mapuje nejkratší cestu od cíle k poèátku a vyresluje trasu
    start = End.parent;           % Promìnná start si zapíše index parent bodu
    line([End.position(1), nodes(start).position(1)], [End.position(2), nodes(start).position(2)],[End.position(3), nodes(start).position(3)], 'Color', 'r', 'LineWidth', 2.5);
    hold on
    targets = [targets; End.position(1), End.position(2),End.position(3)];
    End = nodes(start);           % Zde je index použit, aby pro další cyklus byl End právì tento parent bod         
end                               % Tento cyklus trvá než je parent 0, neboli startovní bod

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


%% Kolizní funkce 


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


