import torch
from torch.utils.data import Dataset
from torch_geometric.io import write_off, read_obj

from os import listdir
from os.path import isfile, join

import torch
from torch_geometric.data import Dataset, download_url, Data
from torch_geometric.io import read_obj
from torch_geometric.transforms import FaceToEdge


class FacesDataset(Dataset):
    def __init__(self, path):
        super(FacesDataset, self).__init__(None)
        self.path = path
        self.filenames = [f for f in listdir(self.path) if isfile(join(self.path, f))]
        self.addEdges = FaceToEdge(remove_faces=False)

    def __len__(self):
        return len(self.filenames)

    def __getitem__(self, index):
        data = read_obj(self.path + self.filenames[index])
        data = self.addEdges(data)
        data.x = data.pos
        return data
