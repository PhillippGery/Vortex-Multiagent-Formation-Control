function [A, L, lambda_2, edges] = compute_laplacian(p_v_all, d_th)
    % p_v_all: 2xN matrix of all virtual look-ahead points
    % d_th: maximum communication distance
    
    N = size(p_v_all, 2);
    A = zeros(N, N);
    edges = [];
    
    % Build Adjacency Matrix and Edge List
    for i = 1:N
        for j = i+1:N
            dist = norm(p_v_all(:, i) - p_v_all(:, j));
            if dist < d_th
                A(i, j) = 1;
                A(j, i) = 1;
                edges = [edges; i, j]; % Store the active connection
            end
        end
    end
    
    % Degree and Laplacian Matrices
    D = diag(sum(A, 2));
    L = D - A;
    
    % Calculate algebraic connectivity (2nd smallest eigenvalue)
    eigenvalues = sort(eig(L));
    if N > 1
        lambda_2 = eigenvalues(2);
    else
        lambda_2 = 0;
    end
end