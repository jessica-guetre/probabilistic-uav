function roi = getTarget(probabilityGrid)
    probabilities = probabilityGrid / sum(probabilityGrid, 'all'); % normalize to sum to 1
    cumulativeProbabilities = cumsum(probabilities(:)); % converto to cumulative sum array
    randValue = rand(); % generate a random value and find the first index where the cumulative
    targetCellIndex = find(cumulativeProbabilities >= randValue, 1, 'first'); % select index for first occurrence
    [row, col] = ind2sub(size(probabilityGrid), targetCellIndex); % convert the 1D index back to 2D
    if row == size(probabilityGrid, 1)
        row = row - 1;
    end
    if col == size(probabilityGrid, 2)
        col = col - 1;
    end
    roi = [row, row + 1, col, col + 1, 0, 3]; % define ROI
end