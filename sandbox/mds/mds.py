from sklearn.datasets import load_digits
from sklearn.manifold import MDS
import numpy as np

nodes = ["DW0001", "DW0002", "DW0003", "DW0004"]

pairwise = np.zeros([len(nodes), len(nodes)])

pairwise[nodes.index('DW0002'), nodes.index('DW0001')]


print(pairwise)
embedding = MDS(n_components=2, normalized_stress='auto', metric=False)
X_transformed = embedding.fit_transform(X[:100])