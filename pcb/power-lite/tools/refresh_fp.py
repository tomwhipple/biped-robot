"""Reload footprints on the routed board from their libraries (run with KiCad's python), for a footprint change
that keeps every pad: the silkscreen, fab outline, courtyard and 3D model change, the copper does not, so the
board needs no re-route. Each footprint is rebuilt as tools/gen_pcb.py builds it (from tools/parts.py); the
pads must come out where they were, with the same size, shape and drill, or nothing is written. Pads keep their
net and zone connection, footprints and pads their UUIDs; tracks and pours are not touched.
usage: refresh_fp.py <board> <ref> ...   (from pcb/power-lite)"""
import sys
import uuid
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_pcb as G  # noqa: E402
import parts as PD  # noqa: E402

NS = uuid.UUID("5f0e2c8a-41b6-4f0b-9a57-3d2e8c1b7a90")


def pad_key(pad):
    return (pad.GetPosition(), pad.GetSize(pcbnew.F_Cu), pad.GetShape(pcbnew.F_Cu), pad.GetDrillSize(),
            pad.GetOrientation().AsDegrees(), pad.GetAttribute(), len(pad.GetLayerSet().Seq()))


def main():
    board_path, refs = sys.argv[1], sys.argv[2:]
    board = pcbnew.LoadBoard(board_path)
    for ref in refs:
        old = board.FindFootprintByReference(ref)
        assert old is not None, ref
        was = old.GetFPIDAsString()
        pos, rot, fp_uuid = old.GetPosition(), old.GetOrientationDegrees(), old.m_Uuid
        old_pads = {p.GetNumber(): (pad_key(p), p.GetNet(), p.GetLocalZoneConnection(), p.m_Uuid) for p in old.Pads()}
        old_fields = {f.GetName(): f.m_Uuid for f in old.GetFields()}
        path = old.GetPath().AsString().lstrip("/")
        board.Remove(old)   # first: the new one takes over its UUIDs

        d = PD.parts[ref]
        new = G.load_fp(ref, dict(fp=d["fp"], value=d["value"], uuid=path, datasheet=d["datasheet"], desc=d["desc"],
                                  mpn=d["mpn"]))
        new.SetUuid(fp_uuid)
        for f in new.GetFields():
            if f.GetName() in old_fields:
                f.SetUuid(old_fields[f.GetName()])
        for i, item in enumerate(new.GraphicalItems()):   # stable UUIDs, so a re-run writes the same file
            item.SetUuid(pcbnew.KIID(str(uuid.uuid5(NS, f"{ref}/{i}"))))
        for pad in new.Pads():
            pad.SetUuid(old_pads[pad.GetNumber()][3])
        board.Add(new)
        G.place(new, ref)
        assert new.GetPosition() == pos and new.GetOrientationDegrees() == rot, f"{ref}: not where the board has it"
        for pad in new.Pads():
            key, net, zc, _ = old_pads.pop(pad.GetNumber())
            if pad_key(pad) != key:
                sys.exit(f"{ref} pad {pad.GetNumber()}: the new footprint moves or reshapes it; re-route instead")
            pad.SetNet(net)
            pad.SetLocalZoneConnection(zc)
        assert not old_pads, f"{ref}: pads missing from the new footprint: {sorted(old_pads)}"
        print(f"{ref}: {was} -> {new.GetFPIDAsString()}, pads unchanged")
    pcbnew.SaveBoard(board_path, board)


if __name__ == "__main__":
    main()
