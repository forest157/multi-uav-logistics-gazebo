"""Summaries for persisted perception-track reliability metrics."""
import json


def summarize_tracks(payload):
    tracks=list((payload or {}).get("obstacles") or [])
    confidences=[max(0.0,min(1.0,float(track.get("confidence",0.0)))) for track in tracks]
    occlusions=[max(0.0,float(track.get("occluded_for_s",0.0))) for track in tracks]
    return {
        "track_count":len(tracks),
        "observed_track_count":sum(1 for track in tracks if track.get("observed",True)),
        "mean_track_confidence":round(sum(confidences)/len(confidences),3) if confidences else 0.0,
        "max_occlusion_s":round(max(occlusions),3) if occlusions else 0.0,
        "track_ids":json.dumps(sorted(str(track.get("id","")) for track in tracks),ensure_ascii=False,separators=(",",":")),
    }


def summarize_recorded_rows(rows):
    rows=list(rows)
    tracked=[row for row in rows if int(float(row.get("track_count") or 0))>0]
    confidences=[float(row.get("mean_track_confidence") or 0.0) for row in tracked]
    identities=set()
    for row in tracked:
        try:identities.update(json.loads(row.get("track_ids") or "[]"))
        except (TypeError,ValueError):pass
    cycles=[];current=set()
    for row in rows:
        if int(float(row.get("track_count") or 0))<=0:
            if current:cycles.append(current);current=set()
            continue
        try:current.update(str(identity) for identity in json.loads(row.get("track_ids") or "[]"))
        except (TypeError,ValueError):pass
    if current:cycles.append(current)
    continuous=sum(1 for cycle in cycles if len(cycle)==1)
    return {
        "tracked_samples":len(tracked),
        "observed_samples":sum(int(float(row.get("observed_track_count") or 0)) for row in tracked),
        "occluded_samples":sum(1 for row in tracked if int(float(row.get("observed_track_count") or 0))==0),
        "mean_confidence":round(sum(confidences)/len(confidences),3) if confidences else 0.0,
        "maximum_occlusion_s":round(max([float(row.get("max_occlusion_s") or 0.0) for row in tracked] or [0.0]),3),
        "unique_track_ids":sorted(identities),
        "visibility_cycles":len(cycles),
        "continuous_visibility_cycles":continuous,
        "id_switches_within_visibility_cycles":sum(max(0,len(cycle)-1) for cycle in cycles),
        "id_continuity_rate":round(float(continuous)/len(cycles),3) if cycles else 0.0,
    }
