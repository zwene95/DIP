function missDistance = computeMissDistance(problem)



    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
            ];
        
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];

    missDistance = min(sqrt(sum(( pIOO - pDOO ).^2)));
  
end
