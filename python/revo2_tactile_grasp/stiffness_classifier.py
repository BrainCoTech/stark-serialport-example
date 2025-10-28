import torch.nn as nn

# ---------------- LSTM/GRU 刚度分类器 ----------------
class StiffnessClassifier(nn.Module):
    def __init__(self, input_size=6, hidden_size=64, num_classes=12, dropout=0.5, bidirectional=True):
        super().__init__()

        self.lstm = nn.LSTM(input_size, hidden_size, num_layers=1,
                            batch_first=True, dropout=0.5, bidirectional=True)
        self.dropout = nn.Dropout(dropout)
        self.bidirectional = bidirectional
        self.fc = nn.Linear(hidden_size*2*(2 if self.bidirectional else 1), num_classes)

    def forward(self, x):
        # x shape: (B, T, 3, 2) -> (B, T, 6)
        B, T, F1, F2 = x.shape
        x = x.view(B, T, F1 * F2)
        out, _ = self.lstm(x)  # (batch, T, hidden*D)
        out = out[:, -1, :]   # 取最后时间步
        # out = self.dropout(out)
        out = self.fc(out)
        return out


